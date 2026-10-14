import math
import time
import requests
import argparse

# Configurations
PI_IP: str = "127.0.0.1"
PI_PORT: int = 80

def play_episode(dataset_name: str, episode_id: int):
    response = requests.post(f"http://{PI_IP}:{PI_PORT}/recording/play", json=dict(
            dataset_name=dataset_name,
            episode_id=episode_id,
        ))
    response.raise_for_status()
    return response.json()

def call_to_api(endpoint: str, data: dict = {}):
    response = requests.post(f"http://{PI_IP}:{PI_PORT}/move/{endpoint}", json=data)
    response.raise_for_status()
    return response.json()

def main():

   parser = argparse.ArgumentParser()
   parser.add_argument("--dataset_name", type=str, default="face_dataset", required=True)
   parser.add_argument("--episode_id", type=int, default=1, required=True)
   args = parser.parse_args()

   call_to_api("init")
   print("Initializing robot")
   time.sleep(2)

   print(f"Playing episode {args.episode_id} of dataset {args.dataset_name}")
   play_episode(dataset_name=args.dataset_name, episode_id=args.episode_id)


if __name__ == "__main__":
    main()

