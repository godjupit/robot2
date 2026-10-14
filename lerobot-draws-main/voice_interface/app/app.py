from flask import Flask, render_template, request, jsonify,send_file, json
import tempfile
import os
from openai import OpenAI
import requests



app = Flask(__name__)



@app.route("/voice-to-prompt", methods=["POST"])
def voice_to_prompt():
    try:
        # Check if audio file is in the request
        if 'audio' not in request.files:
            return jsonify({"error": "No audio file provided"}), 400

        audio_file = request.files['audio']
        if audio_file.filename == '':
            return jsonify({"error": "No file selected"}), 400

        # Create a temporary file to save the uploaded audio
        with tempfile.NamedTemporaryFile(delete=False, suffix='.webm') as temp_file:
            audio_file.save(temp_file.name)
            temp_file_path = temp_file.name

        # Initialize OpenAI client
        client = OpenAI(api_key=os.environ["OPENAI_API_KEY"])

        # Read the temporary file for transcription
        with open(temp_file_path, "rb") as audio:
            transcription = client.audio.transcriptions.create(
                model="whisper-1",  # Using whisper-1 for transcription
                file=audio
            )

        # Clean up temporary file
        os.unlink(temp_file_path)

        # Return the transcription
        return jsonify({
            "transcription": transcription.text,
            "message": "Transcription successful"
        }), 200

    except Exception as e:
        print(e)
        # Clean up in case of error
        if 'temp_file_path' in locals():
            try:
                os.unlink(temp_file_path)
            except:
                pass
        return jsonify({"error": str(e)}), 500

@app.route("/respond-to-user", methods=["POST"])
def respond_to_user():
    try:
        # Initialize OpenAI client
        client = OpenAI(api_key=os.environ["OPENAI_API_KEY"])

        # Get transcription from request
        data = request.get_json()
        if not data or 'transcription' not in data:
            return jsonify({"error": "No transcription provided"}), 400

        transcription = data['transcription']
        if not transcription.strip():
            return jsonify({"error": "Transcription is empty"}), 400

        # Create a temporary file for the MP3
        with tempfile.NamedTemporaryFile(delete=False, suffix='.mp3') as temp_file:
            temp_file_path = temp_file.name
            
        # Say what the user
        
    
        reconfiguredPrompt = client.chat.completions.create(
              model="o4-mini",
              reasoning_effort="medium",
              messages=[
                  {
                      "role": "user", 
                      "content": f"Du bist ein Roboter mit einem deutschen Akzent. Du hast mehrere Roboterarme und ein Mensch möchte dir eine Aufgae geben. Du musst kurz etwas zur Aufgae sagen. Das ist die Input Prompt {transcription}. Du musst kurz zusammenfassen was die Aufgabe ist. Gehe danach darauf ein was du zeichen möchtest. Markiere diesen Abschnitt mit folgendem Code 91919 zum Start und zum Ende. Danach  beschreibe wie du es mit einem Stift und deinem Roboterarm umsetzen möchtest zmb. Ich werde nun einen Stift nehmen um folgendes Bild zu zeichen, zuerst zeichne ich jenes und folgendes... Antworte in der Srpache mit der eine Person mit dir redet."
                  }
              ]
          )
        
        imageDescription = reconfiguredPrompt.choices[0].message.content.split("91919")[1].split("91919")[0]
        # a_simple_house_with_a_door_and_image.png
        imageDescription = "a_simple_house_with_a_door_and_image.png"
        
        print(imageDescription, "Image Description")
        
        # Here we generate the image with the description and call the function, we get back 
        # back the image that is generated! 
        #main_function(imageDescription)
        # CANT FIGURE OUT TO CALL IT PROPERLY 
        # THE IMAGE FILE IS THE PROMPT .png 
        
        
        

        reconfiguredPrompt = reconfiguredPrompt.choices[0].message.content

      

        # Generate speech response
        with client.audio.speech.with_streaming_response.create(
            model="tts-1",
            voice="alloy",
            input=f"{reconfiguredPrompt}.",
            instructions="Speak in a cheerful and positive tone with an strong German Accent (Bavarian)"
        ) as response:
            response.stream_to_file(temp_file_path)
            
        metadata = {
          "imageUrl": imageDescription,
          "message": "File is ready for download"
        }

        # Return the MP3 file
        response = send_file(
            temp_file_path,
            mimetype='audio/mpeg',
            as_attachment=True,
            download_name=f'{imageDescription}.mp3',
        )
        response.headers['X-Metadata'] = json.dumps(metadata) 
        
        return response

    except Exception as e:
        print(e)
        # Clean up in case of error
        if 'temp_file_path' in locals():
            try:
                os.unlink(temp_file_path)
            except:
                pass
        return jsonify({"error": str(e)}), 500



@app.route("/draw-object", methods=["POST"])
def draw_object():
    try:
        print("Starting draw object request...")
        
        # Make POST request to the recording/play endpoint
        response = requests.post(
            'http://0.0.0.0/recording/play',
            headers={
                'accept': 'application/json',
                'Content-Type': 'application/json'
            },
            json={
                
            },
            timeout=30  # Add timeout to prevent hanging
        )
        
        # Check if request was successful
        response.raise_for_status()
        
        # Log successful response
        print(f"Draw object request successful: {response.json()}")
        
        # Return the response data
        return jsonify({
            'status': 'success',
            'data': response.json()
        })
        
    except requests.exceptions.Timeout:
        print("Request timed out")
        return jsonify({
            'status': 'error',
            'error': 'Request timed out'
        }), 504
        
    except requests.exceptions.ConnectionError:
        print("Connection error")
        return jsonify({
            'status': 'error',
            'error': 'Could not connect to the drawing service'
        }), 503
        
    except requests.exceptions.RequestException as e:
        print(f"Request error: {str(e)}")
        return jsonify({
            'status': 'error',
            'error': str(e)
        }), 500

@app.route("/erase-board", methods=["POST"])               # root URL → our only page
def erase_board():
    print("Erasing board")
    
    


@app.route("/")               # root URL → our only page
def home():
    return render_template("home.html")

if __name__ == "__main__":
    # debug=True reloads on code changes – handy while developing
    app.run(debug=True, port=5000)  





