from .color_segmentation import segment_lanes
from .midlane_estimation import estimate_midlane
from ...config import config
from .cleaning import GetYellowInnerEdge, ExtendShortLane
from .data_extraction import FetchInfoAndDisplay
import cv2


def detect_lanes(img):
    #cropping the roi (eg. keeping only below the horizon)
    if img is None or img.size == 0:
        raise ValueError("Image is not loaded correctly")
    
    img_cropped = img[config.CropHeight_resized:,:]

    mid_lane_mask,mid_lane_edge,outer_lane_edge,outerlane_side_sep,outerlane_points = segment_lanes(img_cropped, config.minArea_resized)
    
    estimated_midlane = estimate_midlane(mid_lane_edge, config.MaxDist_resized)

    #Cleaning stage Step 1: Getting the outerlane close to the midlane
    OuterLane_OneSide, Outer_cnts_OneSide, Mid_cnts, Offset_correction = GetYellowInnerEdge(outerlane_side_sep, estimated_midlane, outerlane_points)
    #Cleaning stage step 2: Extending the short lane
    extended_midlane, extended_outerlane = ExtendShortLane(estimated_midlane, Mid_cnts, Outer_cnts_OneSide, OuterLane_OneSide.copy())

    Distance , Curvature = FetchInfoAndDisplay(mid_lane_edge,extended_midlane,extended_outerlane,img_cropped,Offset_correction)

    
    cv2.imshow("mid_lane_mask", mid_lane_mask)
    cv2.imshow("mid_lane_edge", mid_lane_edge)
    cv2.imshow("outer_lane_edge", outer_lane_edge)
    cv2.imshow("outerlane_side_sep", outerlane_side_sep)
    cv2.imshow("estimated_midlane", estimated_midlane)
    cv2.imshow("OuterLane_OneSide", OuterLane_OneSide)
    cv2.imshow("extended_midlane", extended_midlane)
    cv2.imshow("extended_outerlane", extended_outerlane)

    cv2.waitKey(1)
    return Distance, Curvature
    

