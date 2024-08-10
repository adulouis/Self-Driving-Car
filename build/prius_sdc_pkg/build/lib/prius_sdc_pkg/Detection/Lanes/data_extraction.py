import cv2
import numpy as np
from .utilities import Cord_Sort, findlaneCurvature


def LanePoints(midlane, outerlane, offset):
    mid_cnts = cv2.findContours(midlane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    outer_cnts = cv2.findContours(outerlane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

    if mid_cnts and outer_cnts:
        mid_cnts_row_sorted = Cord_Sort(mid_cnts, "rows")
        outer_cnts_row_sorted = Cord_Sort(outer_cnts, "rows")

        m_rows = mid_cnts_row_sorted.shape[0]
        o_rows = outer_cnts_row_sorted.shape[0]

        #Getting the bottom points
        m_rows_btm_point = mid_cnts_row_sorted[m_rows-1,:]
        o_rows_btm_point = outer_cnts_row_sorted[o_rows-1,:]

        #Getting the top points
        m_rows_top_point = mid_cnts_row_sorted[0,:]
        o_rows_top_point = outer_cnts_row_sorted[0,:]

        #Estimating the trajectory
        traj_btm_point = (int((m_rows_btm_point[0] + o_rows_btm_point[0])/2)+ offset, int((m_rows_btm_point[1] + o_rows_btm_point[1])/2))
        traj_top_point = (int((m_rows_top_point[0] + o_rows_top_point[0])/2)+ offset, int((m_rows_top_point[1] + o_rows_top_point[1])/2))

        return traj_btm_point, traj_top_point
    
    else:
        return (0,0), (0,0)
    

def EstimateNonMidMask(MidEdgeROi):
    Mid_Hull_Mask = np.zeros((MidEdgeROi.shape[0], MidEdgeROi.shape[1], 1), dtype=np.uint8)
    contours = cv2.findContours(MidEdgeROi,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)[0]
    if contours:
        hull_list = []
        contours = np.concatenate(contours)
        hull = cv2.convexHull(contours)
        hull_list.append(hull)
        Mid_Hull_Mask = cv2.drawContours(Mid_Hull_Mask, hull_list, 0, 255,-1)
    Non_Mid_Mask=cv2.bitwise_not(Mid_Hull_Mask)
    return Non_Mid_Mask
    

def FetchInfoAndDisplay(Mid_lane_edge, Mid_lane, Outer_Lane, frame, Offset_correction):
    #Using both outer and middle information to create probable path
    Traj_lowP, Traj_highP = LanePoints(Mid_lane, Outer_Lane, Offset_correction)

    #Compute Distance and Curvature from the Trajectory Points
    PerpDist_LaneCentralStart_CarNose = -1000
    if (Traj_lowP!=(0,0)):
        PerpDist_LaneCentralStart_CarNose = Traj_lowP[0] - int(Mid_lane.shape[1]/2)
    curvature = findlaneCurvature(Traj_lowP[0], Traj_lowP[1], Traj_highP[0], Traj_highP[1])

    #Keep only those edge that are part of Midlane
    Mid_lane_edge = cv2.bitwise_and(Mid_lane_edge, Mid_lane)

    #Combine Mid and OuterLane to get Lanes Combined and extract its contours
    Lanes_combined = cv2.bitwise_or(Outer_Lane, Mid_lane)
    cv2.imshow("Lanes combined", Lanes_combined)
    ProjectedLane = np.zeros(Lanes_combined.shape, Lanes_combined.dtype)
    cnts = cv2.findContours(Lanes_combined, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]

    #Fill ProjectedLane
    if cnts:
        cnts = np.concatenate(cnts)
        cnts = np.array(cnts)
        cv2.fillConvexPoly(ProjectedLane, cnts, 255)
    
    #Remove Midlane_Region from ProjectedLane by extracting the midless mask
    Mid_less_Mask = EstimateNonMidMask(Mid_lane_edge)
    ProjectedLane = cv2.bitwise_and(Mid_less_Mask, ProjectedLane)

    # 7. Draw projected lane
    Lane_drawn_frame = frame
    Lane_drawn_frame[ProjectedLane==255] = Lane_drawn_frame[ProjectedLane==255] + (0,100,0)
    Lane_drawn_frame[Outer_Lane==255] = Lane_drawn_frame[Outer_Lane==255] + (0,0,100)# Outer Lane Coloured Red
    Lane_drawn_frame[Mid_lane==255] = Lane_drawn_frame[Mid_lane==255] + (100,0,0)# Mid Lane Coloured Blue
    Out_image = Lane_drawn_frame

    #Draw Cars direction and Lanes directon and distance between care and lane path
    cv2.line(Out_image,(int(Out_image.shape[1]/2),Out_image.shape[0]),(int(Out_image.shape[1]/2),Out_image.shape[0]-int(Out_image.shape[0]/5)),(0,0,255),2)
    cv2.line(Out_image, Traj_lowP, Traj_highP,(255,0,0),2)

    if (Traj_lowP!=(0,0)):
        cv2.line(Out_image,Traj_lowP,(int(Out_image.shape[1]/2),Traj_lowP[1]),(255,255,0),2) #distance of car center with lane

    #Draw extracted distance and curvature
    curvature_str = "Curvature = " + f"{curvature:.2f}"
    PerpDist_ImgCen_CarNose_str = "Distance = " + str(PerpDist_LaneCentralStart_CarNose)
    textSize_ratio = 0.5
    cv2.putText(Out_image,curvature_str,(10,30),cv2.FONT_HERSHEY_DUPLEX,textSize_ratio,(0,255,255),1)
    cv2.putText(Out_image, PerpDist_ImgCen_CarNose_str,(10,50),cv2.FONT_HERSHEY_DUPLEX,textSize_ratio,(0,255,255),1)
    return PerpDist_LaneCentralStart_CarNose, curvature
