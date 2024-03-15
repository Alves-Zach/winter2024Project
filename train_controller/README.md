# Train controler
Launch the controller with the sign_detection node with:
`ros2 launch train_controller train.launch.py`

This launches two nodes:

* **sign_detection**

    * Identifies the signs seen by the camera, and publishes those names to the */sign* topic

* **controler**

    * Subscribes to the */sign* topic and commands the PX100 to move to the position associated with the command

        * Fast
        * Slow
        * Stop
        * Switch tracks