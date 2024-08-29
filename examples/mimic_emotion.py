
#!/usr/bin/env python

import rospy
from pyhri import HRIListener
from hri_msgs.msg import Expression

def main():
    # Initialize the ROS node
    rospy.init_node("emotion_matcher")

    # Create an instance of HRIListener to listen for HRI events
    hri_listener = HRIListener()

    # Create a publisher to publish the robot's expression
    expression_pub = rospy.Publisher(
        "/robot_face/expression",
        Expression,
        queue_size=1,
    )

    # Define a mapping of user emotions to robot expressions
    expression_to_string = {
      Expression.HAPPY: "happy",
      Expression.ANGRY: "angry",
      Expression.SAD: "sad",
      Expression.SURPRISED: "surprised",
      Expression.DISGUSTED: "disgusted",
      Expression.SCARED: "scared",
      Expression.NEUTRAL: "neutral"
    }
    # Loop to continuously check for tracked persons
    while not rospy.is_shutdown():
        # Iterate through the tracked persons
        for face_id, face in hri_listener.faces.items():
            if face.expression:
                # Get the user's expression
                user_expression = face.expression.expression
                # Map user expression to robot expression
                robot_expression = expression_to_string.get(user_expression, "neutral")
                # Create the message for the robot's expression
                expression_msg = Expression(
                    expression=robot_expression,
                )

                # Publish the robot's expression
                expression_pub.publish(expression_msg)
                rospy.loginfo(f"User expression: {user_expression}. Robot expression set to: {robot_expression}.")
            else:
                rospy.loginfo(f"Person {face_id} has no associated expression.")

        # Sleep for a short duration to avoid high CPU usage
        rospy.sleep(1)

if __name__ == "__main__":
    main()
