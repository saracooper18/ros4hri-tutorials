import rospy
from pyhri import HRIListener

def main():
    # Initialize the ROS node
    rospy.init_node("person_listing")
    
    # Create an instance of HRIListener to listen for HRI events
    hri_listener = HRIListener()
    
    # Loop to continuously check for tracked persons
    while not rospy.is_shutdown():
        # Iterate through the tracked persons
        for person_id, person in hri_listener.tracked_persons.items():
            # Check if the person has an associated face
            if person.face:
                # Print the person's ID and the associated face's ID
                print(f"Person {person_id} is bound to face {person.face.id}")
                print("6D gaze orientation: %s" % person.face.gaze_transform(from_frame="head_camera"))
            else:
                # Print the person's ID if no face is associated
                print(f"Person {person_id} has no associated face")
        
        # Sleep for a short duration to avoid high CPU usage
        rospy.sleep(1)
if __name__ == "__main__":
    main()
