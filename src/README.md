# 💻Source folder
This folder contains all the Python files to run the project

## Structure of the folder
- **pc** Codes that should be executed in the PC where the LLM is executing.
- **raspberry** Codes to be executed in the Raspberry on-board.

## Raspberry Pi
### 📋Requirements
In order to execute successfully these codes, you will need:
- A Raspberry Pi 4 Model B
- Assembled the robot (you can find how to do it [here](assembly-tutorial.md))
- Having installed the requirements.txt file. You can do it by creating a virtual environment and typing:

  ```
  cd raspberry
  pip install -r requirements.txt
  ```

### ▶️Code execution
Once installed the requirements and assembled the robot, the next step is to execute your code. 

To do it, just open a cmd session with admin permission, open your virtual environment and write:

```
cd raspberry
python robot_code_raspberry.py
```

## PC
### 📋Requirements
In order to execute your local server in your computer, you will need:
- Having installed **LMStudio**. You can find how to install it [here](https://lmstudio.ai/download).
- Inside LMStudio, you must have installed the models to use. In this example it is used Gemma 3 4B.
- Having configured the model with the custom preset provided. You can find it [here](src/pc/Gemma3-4B.preset.json). In this preset it is defined the system prompt, apart from other parameters such as the model temperature.
- Having installed the requirements.txt file. You can do it by creating a virtual environment and typing:

  ```
  cd pc
  pip install -r requirements.txt
  ```

### ▶️Code execution
Once done all the steps above, the next step is to execute your local server. This will wait an image captured by the robot and then will call the LLM and return the response.

```
cd raspberry
python robot_code_pc.py
```

This will print the URL of the server. You will have to copy this and paste it into the Raspberry code where specified.
