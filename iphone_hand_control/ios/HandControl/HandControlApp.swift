import SwiftUI
import AVFoundation
import Vision
import simd

@main
struct HandControlApp: App {
    var body: some Scene { WindowGroup { ControlView() } }
}

struct ControlView: View {
    @StateObject private var tracker = HandTracker()
    @Environment(\.scenePhase) private var phase
    @State private var address = "http://192.168.1.100:8765"
    @State private var token = ""
    @State private var following = false
    var body: some View {
        VStack(spacing: 12) {
            Text("SO100 手部位置控制").font(.title2)
            HStack {
                TextField("电脑地址", text: $address).textInputAutocapitalization(.never).autocorrectionDisabled()
                SecureField("终端显示的 Token", text: $token)
            }.textFieldStyle(.roundedBorder).disabled(following)
            Text(tracker.status).font(.system(.body, design: .monospaced))
            HStack {
                Button(following ? "暂停跟随" : "开始跟随 / 重新定零") {
                    following.toggle()
                    tracker.configure(address: address, token: token, enabled: following)
                }.buttonStyle(.borderedProminent)
                Button("停止") {
                    following = false
                    tracker.configure(address: address, token: token, enabled: false)
                }.buttonStyle(.bordered).tint(.red)
            }
            Text("固定手机并横屏；手掌朝向前置相机。首个有效位置设为原点。\n左右 / 上下 / 远近控制末端位置；不控制夹爪。")
                .font(.footnote)
        }.padding().onAppear { tracker.start() }
        .onChange(of: phase) { _, value in
            if value != .active {
                following = false
                tracker.configure(address: address, token: token, enabled: false)
            }
        }
    }
}

final class HandTracker: NSObject, ObservableObject, AVCaptureDataOutputSynchronizerDelegate {
    @Published var status = "正在申请相机权限"
    private let queue = DispatchQueue(label: "hand.capture")
    private let session = AVCaptureSession()
    private let video = AVCaptureVideoDataOutput()
    private let depth = AVCaptureDepthDataOutput()
    private var synchronizer: AVCaptureDataOutputSynchronizer?
    private let request = VNDetectHumanHandPoseRequest()
    private var endpoint: URL?
    private var token = ""
    private var enabled = false
    private var busy = false
    private var lastSent = 0.0
    private var started = false
    private let network: URLSession = {
        let c = URLSessionConfiguration.ephemeral
        c.timeoutIntervalForRequest = 0.5
        c.timeoutIntervalForResource = 0.7
        return URLSession(configuration: c)
    }()

    private func report(_ text: String) {
        DispatchQueue.main.async { self.status = text }
    }

    func configure(address: String, token: String, enabled: Bool) {
        queue.async {
            self.endpoint = URL(string: address.trimmingCharacters(in: .whitespacesAndNewlines) + "/hand")
            self.token = token
            self.enabled = enabled
            if !enabled { self.send(nil) }
        }
    }

    func start() {
        AVCaptureDevice.requestAccess(for: .video) { allowed in
            guard allowed else { self.report("请在设置中允许相机访问"); return }
            self.queue.async {
                guard !self.started else { return }
                self.started = true
                do { try self.setup() } catch { self.report("相机启动失败: \(error.localizedDescription)") }
            }
        }
    }

    private func setup() throws {
        guard let camera = AVCaptureDevice.default(.builtInTrueDepthCamera, for: .video, position: .front) else {
            throw NSError(domain: "TrueDepth unavailable", code: 1)
        }
        session.beginConfiguration()
        session.sessionPreset = .inputPriority
        let input = try AVCaptureDeviceInput(device: camera)
        guard session.canAddInput(input) else { throw NSError(domain: "Camera input", code: 2) }
        session.addInput(input)
        try camera.lockForConfiguration()
        let formats = camera.formats.filter { !$0.supportedDepthDataFormats.isEmpty }
        guard let format = formats.min(by: {
            CMVideoFormatDescriptionGetDimensions($0.formatDescription).width < CMVideoFormatDescriptionGetDimensions($1.formatDescription).width
        }), let df = format.supportedDepthDataFormats.last else {
            camera.unlockForConfiguration()
            throw NSError(domain: "Depth format unavailable", code: 3)
        }
        camera.activeFormat = format
        camera.activeDepthDataFormat = df
        camera.unlockForConfiguration()
        video.alwaysDiscardsLateVideoFrames = true
        video.videoSettings = [kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA]
        depth.isFilteringEnabled = false
        guard session.canAddOutput(video), session.canAddOutput(depth) else {
            throw NSError(domain: "Synchronized outputs unavailable", code: 4)
        }
        session.addOutput(video)
        session.addOutput(depth)
        // Native landscape buffers and calibration coordinates; no selfie mirroring.
        for connection in [video.connection(with: .video), depth.connection(with: .depthData)].compactMap({ $0 }) {
            if connection.isVideoOrientationSupported { connection.videoOrientation = .landscapeRight }
            if connection.isVideoMirroringSupported {
                connection.automaticallyAdjustsVideoMirroring = false
                connection.isVideoMirrored = false
            }
        }
        synchronizer = AVCaptureDataOutputSynchronizer(dataOutputs: [video, depth])
        synchronizer?.setDelegate(self, queue: queue)
        session.commitConfiguration()
        request.maximumHandCount = 1
        session.startRunning()
        report("相机已启动；填写电脑地址和 Token 后开始跟随")
    }

    func dataOutputSynchronizer(_ synchronizer: AVCaptureDataOutputSynchronizer,
                                didOutput collection: AVCaptureSynchronizedDataCollection) {
        guard enabled else { return }
        guard let rgb = collection.synchronizedData(for: video) as? AVCaptureSynchronizedSampleBufferData,
              let dep = collection.synchronizedData(for: depth) as? AVCaptureSynchronizedDepthData,
              !rgb.sampleBufferWasDropped, !dep.depthDataWasDropped else {
            send(nil); return
        }
        do {
            try VNImageRequestHandler(cmSampleBuffer: rgb.sampleBuffer, orientation: .up).perform([request])
            guard let hand = request.results?.first else { send(nil); return }
            let points = try hand.recognizedPoints(.all)
            let names: [VNHumanHandPoseObservation.JointName] = [.wrist, .indexMCP, .middleMCP, .ringMCP, .littleMCP]
            let palm = names.compactMap { points[$0] }.filter { $0.confidence > 0.6 }
            guard palm.count == names.count else { send(nil); return }
            let u = palm.map { $0.location.x }.reduce(0, +) / Double(palm.count)
            let v = 1 - palm.map { $0.location.y }.reduce(0, +) / Double(palm.count)
            let data = dep.depthData.converting(toDepthDataType: kCVPixelFormatType_DepthFloat32)
            guard let calibration = data.cameraCalibrationData else { send(nil); return }
            let map = data.depthDataMap
            CVPixelBufferLockBaseAddress(map, .readOnly)
            defer { CVPixelBufferUnlockBaseAddress(map, .readOnly) }
            let w = CVPixelBufferGetWidth(map), h = CVPixelBufferGetHeight(map)
            let px = Int(u * Double(w)), py = Int(v * Double(h))
            guard px >= 2, py >= 2, px < w-2, py < h-2,
                  let base = CVPixelBufferGetBaseAddress(map) else { send(nil); return }
            let stride = CVPixelBufferGetBytesPerRow(map) / MemoryLayout<Float>.size
            let values = base.assumingMemoryBound(to: Float.self)
            var samples: [Float] = []
            for y in py-2...py+2 {
                for x in px-2...px+2 {
                    let z = values[y*stride+x]
                    if z.isFinite && z > 0.12 && z < 1 { samples.append(z) }
                }
            }
            guard samples.count >= 15 else { send(nil); return }
            samples.sort()
            let z = samples[samples.count/2]
            guard samples.last! - samples.first! < 0.06 else { send(nil); return }
            let k = calibration.intrinsicMatrix
            let ref = calibration.intrinsicMatrixReferenceDimensions
            let x = (Float(u * ref.width) - k.columns.2.x) * z / k.columns.0.x
            let y = (Float(v * ref.height) - k.columns.2.y) * z / k.columns.1.y
            send([x, y, z])
        } catch { send(nil) }
    }

    private func send(_ xyz: [Float]?) {
        // One request in flight, no frame queue. Server also expires samples.
        guard !busy, let endpoint else { return }
        let now = ProcessInfo.processInfo.systemUptime
        guard xyz == nil || now-lastSent >= 0.08 else { return }
        lastSent = now
        var payload: [String: Any] = ["enabled": enabled && xyz != nil]
        if let xyz { payload["xyz"] = xyz }
        var req = URLRequest(url: endpoint)
        req.httpMethod = "POST"
        req.setValue("application/json", forHTTPHeaderField: "Content-Type")
        req.setValue("Bearer \(token)", forHTTPHeaderField: "Authorization")
        req.httpBody = try? JSONSerialization.data(withJSONObject: payload)
        busy = true
        network.dataTask(with: req) { data, response, error in
            self.queue.async {
                self.busy = false
                if let error { self.report("网络: \(error.localizedDescription)"); return }
                guard (response as? HTTPURLResponse)?.statusCode == 200 else {
                    self.report("连接失败：检查地址和 Token"); return
                }
                let message = data.flatMap { String(data: $0, encoding: .utf8) } ?? "已连接"
                self.report(xyz == nil ? "未跟随 / 未检测到有效深度\n\(message)" : message)
                // A stop pressed during the previous request must be delivered next.
                if !self.enabled && xyz != nil { self.send(nil) }
            }
        }.resume()
    }
}
