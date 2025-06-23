import asyncio
import websockets
import base64
import cv2
import numpy as np
import traceback
import os
import datetime
import requests 

# WebSocket server configuration

HOST = "0.0.0.0"
PORT = 8765
SAVE_DIR = "D:\\Descargas" # Change this to your desired save directory

# Create the save directory if it doesn't exist
os.makedirs(SAVE_DIR, exist_ok=True)

def send_request(img_base64: str) -> str:
    """
    Send a request to the model via HTTP API with an image in base64 format.
    """
    try:
        url = "http://localhost:1234/v1/chat/completions"

        headers = {
            "Content-Type": "application/json"
        }

        payload = {
            "model": "gemma-3-4b-it",
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{img_base64}"
                            }
                        },
                        {
                            "type": "text",
                            "text": "OBJETO: pelota"
                        }
                    ]
                }
            ],
            "temperature": 0.2
        }

        response = requests.post(url, headers=headers, json=payload)
        response.raise_for_status()
        data = response.json()

        return data['choices'][0]['message']['content']
    except Exception as e:
        traceback.print_exc()
        return f"Error doing the request: {str(e)}"

async def receive_image(websocket):
    """
    Handle incoming WebSocket connections and process received images.
    """
    try:
        print("Connected to WebSocket server, waiting for image...")
        img_base64 = await websocket.recv()
        print("Received image in base64 format.")

        try:
            # Decode the base64 image
            img_data = base64.b64decode(img_base64)
            nparr = np.frombuffer(img_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if frame is not None:
                print(f"Decoded image. Size: {frame.shape}")

                # Generate a unique filename based on the current timestamp
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                img_filename = f"image_{timestamp}.jpg"
                img_path = os.path.join(SAVE_DIR, img_filename)

                # Guardar la imagen
                cv2.imwrite(img_path, frame)
                print(f"Saved image: {img_path}")
                

                print("Executing model via API...")
                model_response = send_request(img_base64)
                print(f"Model response {model_response}")

                await websocket.send(model_response)
            else:
                print("Error: Image decoding failed.")
                await websocket.send("Error: Image decoding failed.")
        except Exception as e:
            error_msg = f"Error occurred while processing the image: {str(e)}"
            print(error_msg)
            traceback.print_exc()
            await websocket.send(error_msg)
    except Exception as e:
        print(f"Error in the connection: {str(e)}")
        traceback.print_exc()


async def main():
    try:
        server = await websockets.serve(
            receive_image,
            HOST,
            PORT,
            ping_interval=None,
            ping_timeout=None,
            max_size=10485760
        )
        print(f"WebSocket server listening in {HOST}:{PORT}")
        await asyncio.Future()
    except Exception as e:
        print(f"Error occurred while initializing the server: {str(e)}")
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(main())