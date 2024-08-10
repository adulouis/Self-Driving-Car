import cv2
import os
import numpy as np
import math

from ..prius_sdc_pkg.Detection.TrafficLights.colour_segmentation import colour_segment

class TL_States:
    def __init__(self):
        #State Variables
        self.Traffic_State = "Unknown"
        self.prev_Traffic_State = 0
        
        #Control variables
        self.draw_all_detected = True
        self.colour_segmenter = colour_segment()
    
    def get_state(self,center,center_cmp):
        TL_Update = "Unknown"
        #If Center is Brighter
        if( (int(self.colour_segmenter.HLS[center[1],center[0],1]) - int(self.colour_segmenter.HLS[center_cmp[1],center_cmp[0],1])) > 10 ):
            # Left was Brightest [Red]
            if(center[0]<center_cmp[0]):
                TL_Update = "Left was Brightest [Red]"
                self.Traffic_State="Stop"
            # Right was Brightest [Green]
            elif(center[0]>center_cmp[0]):
                TL_Update = "Right was Brightest [Green]"
                self.Traffic_State="Go"

        #ElseIf Center_cmp is Brighter
        elif( ( int(self.colour_segmenter.HLS[center[1],center[0],1]) - int(self.colour_segmenter.HLS[center_cmp[1],center_cmp[0],1]) ) < -10):
            # Left was Darker [Green]
            if(center[0]<center_cmp[0]):
                TL_Update = "Left was Darker [Green]"
                self.Traffic_State="Go"
            # Right was Darker [Red]
            elif(center[0]>center_cmp[0]):
                TL_Update = "Right was Darker [Red]"
                self.Traffic_State="Stop"
        else:
            if (self.prev_Traffic_State != "Stop"):
                self.Traffic_State= "Unknown"#Because No Traffic light is detected and we werent looking for Go then Reset Traffic State        
        

        return TL_Update


    def Confirm_TL_Nd_RetState(self,gray,frame_draw):
        frame_draw_special = frame_draw.copy()
        TL_Update = "Unknown"

        #Apply the HoughCircles to detect the circular regions in the Image
        NumOfVotesForCircle = 16 #parameter 1 MinVotes needed to be classified as circle
        CannyHighthresh = 230
        minDistanceBtwnCircles = 5#kept as sign will likely not be overlapping
        max_rad = 50 #smaller circles dont have enough votes so only maxRadius need to be controlled

        circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,minDistanceBtwnCircles,param1=CannyHighthresh,param2=NumOfVotesForCircle,minRadius=5,maxRadius=max_rad)
        if circles is not None:
            circles = np.uint16(np.around(circles))

        #Check if circles are larger than minim size
            for index, circle in enumerate(circles[0,:]):
                center = (int(circle[0])-1,int(circle[1])-1)
                radius = int(circle[2]+5)

                for index_cmp, circle_cmp in enumerate(circles[0,:]):
                    #Don't compare a circle with itself
                    if index_cmp != index:
                        center_cmp = (int(circle_cmp[0])-1,int(circle_cmp[1])-1)
                        radius_cmp = int(circle_cmp[2]+5)
                        #Check if detected Roi is actually a traffic light or not
                        point_Dist = self.dist(center,center_cmp)

                        if ((point_Dist>10) and (point_Dist<80) and (abs(center[0]-center_cmp[0])<80) and
                            (abs(center[1]-center_cmp[1])<5) and (abs(radius-radius_cmp)<5) and
                            (self.AreCircles_intersecting(center,center_cmp,radius,radius_cmp)<0)):

                            Correct_Color_Comb = self.Check_Color_Cmb(center,center_cmp)
                            
                            #Retrieving the State of the traffic lights
                            if Correct_Color_Comb:
                                TL_Update = self.get_state(center,center_cmp)


    
    def Get_TL_State(self,frame,frame_draw):
        gray_yellow_red_regions = self.colour_segmenter.isolate_yelo_red_regions(frame)

        #Localizing potential candidates and classifying them in sign detection
        self.Confirm_TL_Nd_RetState(gray_yellow_red_regions)

        return self.Traffic_State



class Cascade_Detector:
    def __init__(self):
        print("Initialized Haar Cascade Object Detector")
        self.TL_States = TL_States()

    #Class Variables
    TrafficLight_cascade_str = os.path.join(os.getcwd(), "prius_sdc_pkg/prius_sdc_pkg/data/TrafficLight_cascade.xml")
    TrafficLight_cascade = cv2.CascadeClassifier()

    if not TrafficLight_cascade.load(cv2.samples.findFile(TrafficLight_cascade_str)):
        print('--(!)Error loading face cascade')
        exit(0)

    
    def detect(self,img,img_draw):

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        traffic_light_confirmed=False
        Traffic_State = "Unknown"
        TL_Confirmed_mask = np.zeros_like(gray)

        target = self.TrafficLight_cascade.detectMultiScale(img)

        for (x,y,w,h) in target:
            cv2.rectangle(img_draw, (x,y), (x+w,y+h), (0,165,255), 2)
            TL_Maybe_mask = np.zeros_like(gray)
            TL_Maybe_mask[y:y+h,x:x+w] = 255
            img_ROI = cv2.bitwise_and(img, img, mask=TL_Maybe_mask)
            cv2.imshow("detect Roi", img_ROI)

        #Reconfirm if detected Traffic Light was the desired one
            Traffic_State = self.TL_States.Get_TL_State(img_ROI, img_draw)
            if(Traffic_State!="Unknown"):
                print("Traffic State Received = ", Traffic_State)
                #Confirm Traffic Light
                cv2.rectangle(img_draw, (x,y),(x+w, y+h),(0,255,0),2)
                #Start Tracking
                traffic_light_confirmed = True
                TL_Confirmed_mask = TL_Maybe_mask
                break
        
        return traffic_light_confirmed,TL_Confirmed_mask,Traffic_State,gray


cascade_detector = Cascade_Detector()


def detect_TrafficLight(img,img_draw):

    #Detect trafic light using cascade detector
    tl_confirmed, tl_confirmed_mask, tl_states, gray = cascade_detector.detect(img, img_draw)

    if tl_confirmed:
        #Init Tracker
        print("Confirmed Traffic Light detection ===> (Start Tracking )")

    return tl_states