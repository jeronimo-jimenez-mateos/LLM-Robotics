from robot_package import *

url = "" # URL of the LLM server
path = '/home/usuario/TFM/ejemplo.png'
min_dist_cm = 30 # If the robot finds an object / obstacle at this distance, it won't continue

time.sleep(20)
cap, width, height = initialize_camera()
robot = RobotController()
robot.calibrate_gyro()

while(True):
    start = time.time()
    left_gray, right_gray, left = take_picture(cap, path, width, height)

    image = cv2.imread(path)
    
    response, llm_time = asyncio.run(send_image(url, path))
    end = time.time()
    print(response, end - start)
    response_list = response.split()
    action = response_list[0]
    if action == "EXPLORA":
        robot.explore()
    elif action == "AVANZA":
        rel_distance = response_list[1]
        if rel_distance == "cerca":
            distance = 0.5
        elif rel_distance == "lejos":
            distance = 1
        robot.move_distance (distance, min_dist_cm)

