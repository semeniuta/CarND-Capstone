from styx_msgs.msg import TrafficLight

from tlcv import detect_red_light

class TLClassifier(object):

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        
        # NOTE: In the current form, TLClassifier uses 
        # classical computer vision routines from 
        # tlcv to distinguish between red traffic light 
        # and everyhing else

        if detect_red_light(image):
            return TrafficLight.RED

        return TrafficLight.UNKNOWN
