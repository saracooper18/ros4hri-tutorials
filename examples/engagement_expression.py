import rospy
from pyhri import HRIListener
from hri_msgs.msg import EngagementLevel, Expression
import tf2_ros
from tf import transformations
import math

# Field of 'attention' of a person
FOV = 60.0 * math.pi / 180

# Threshold of 'visual social engagement' to consider engagement
ENGAGEMENT_THR = 0.5

REFERENCE_FRAME = "sellion_link"

listener = None
hri = None
engaged_persons = dict()


class Person:
    def __init__(self, face):
        self.face = face

        # Publisher for the engagement status of the person.
        #For simplicity we will publish at a constant ROS topic. Note "thisisme" will usually be the unique person ID
        # As the tutorial will be done with your laptops cameras, it is likely whole body is not detected, so 
        # we wil only use the face.
        self.engagement_status_pub = rospy.Publisher(
            "/humans/people/thisisme/engagement_status",
            EngagementLevel,
            queue_size=1,
        )

        # Publisher for the robot's facial expression
        self.expression_pub = rospy.Publisher(
            "/robot_face/expression",
            Expression,
            queue_size=1,
        )

        self.level = EngagementLevel.UNKNOWN

    def update_engagement(self):
        """
        Computes the current 'visual social engagement' metric.
        """
        if not self.face:
            rospy.logwarn("No face detected, cannot compute engagement")
            return

        # Compute the person's position 'viewed' from the robot's 'gaze'
        person_from_robot = self.face.gaze_transform(from_frame=REFERENCE_FRAME)
        if person_from_robot == tf2_ros.TransformStamped():
            rospy.logwarn(
                "null transform published for person's face %s" % self.face.id
            )
            return

        # Compute the inverse: the robot 'viewed' from the person's gaze
        trans = [
            person_from_robot.transform.translation.x,
            person_from_robot.transform.translation.y,
            person_from_robot.transform.translation.z,
        ]

        rot = [
            person_from_robot.transform.rotation.x,
            person_from_robot.transform.rotation.y,
            person_from_robot.transform.rotation.z,
            person_from_robot.transform.rotation.w,
        ]

        transform = transformations.concatenate_matrices(
            transformations.translation_matrix(trans),
            transformations.quaternion_matrix(rot),
        )

        robot_from_person = transformations.inverse_matrix(transform)

        # Measure of visual social engagement
        tx, ty, tz = transformations.translation_from_matrix(robot_from_person)
        d_AB = math.sqrt(tx ** 2 + ty ** 2 + tz ** 2)

        # Gaze_AB: How 'close' B is from the optical axis of A
        xB = tz
        yB = tx
        zB = ty

        gaze_AB = 0.0
        if xB > 0:
            gaze_AB = max(
                0, 1 - (math.sqrt(yB ** 2 + zB ** 2) / (math.tan(FOV) * xB)))

        # Gaze_BA: How 'close' A is from the optical axis of B
        tBA = person_from_robot.transform.translation
        xA = tBA.x
        yA = tBA.y
        zA = tBA.z

        gaze_BA = 0.0
        if xA > 0:
            gaze_BA = max(
                0, 1 - (math.sqrt(yA ** 2 + zA ** 2) / (math.tan(FOV) * xA)))

        M_AB = gaze_AB * gaze_BA
        rospy.logdebug("gazeAB: %s, gazeBA=%s,  M_AB: %s" %
                       (gaze_AB, gaze_BA, M_AB))

        S_AB = min(1, M_AB / d_AB)

        rospy.logdebug("S_AB: %s" % S_AB)

        if S_AB < ENGAGEMENT_THR:
            self.level = EngagementLevel.DISENGAGED
        else:
            self.level = EngagementLevel.ENGAGED

    def publish_engagement(self):
        engagement_msg = EngagementLevel()
        engagement_msg.level = self.level
        self.engagement_status_pub.publish(engagement_msg)

        # Publish expression based on engagement level
        expression_msg = Expression(
            valence=0.0,
            arousal=0.0,
            confidence=0.0,
        )
       
        if self.level == EngagementLevel.ENGAGED:
            expression_msg.expression = "happy"
            print("user engaged")
        elif self.level == EngagementLevel.DISENGAGED:
            expression_msg.expression = "sad"
            print("user disengaged")
        else:  # UNKNOWN
            expression_msg.expression = "neutral"

        self.expression_pub.publish(expression_msg)

    def close(self):
        self.level = EngagementLevel.UNKNOWN
        self.publish_engagement()
        self.engagement_status_pub.unregister()
        self.expression_pub.unregister()


def get_tracked_humans():
    # Get the list of IDs of the currently visible persons
    persons = hri.faces.keys()

    # Anyone who disappeared?
    for id in list(engaged_persons.keys()):
        if id not in persons:
            engaged_persons[id].close()
            del engaged_persons[id]

    # Any new person?
    for id, person in hri.faces.items():
        if id not in engaged_persons:
            engaged_persons[id] = Person(person)

        engaged_persons[id].update_engagement()
        engaged_persons[id].publish_engagement()


if __name__ == "__main__":
    nh = rospy.init_node("engagement_detector")

    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    hri = HRIListener()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        get_tracked_humans()
        rate.sleep()