import cv2
import math
from .Morph_op import RetLargestContour


def Distance_(a,b):
    return math.sqrt( ( (a[1]-b[1])**2 ) + ( (a[0]-b[0])**2 ) )

def ApproxDistBWCntrs(cnt,cnt_cmp):
    # compute the center of the contour
    M = cv2.moments(cnt)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    # compute the center of the next contour
    M_cmp = cv2.moments(cnt_cmp)
    cX_cmp = int(M_cmp["m10"] / M_cmp["m00"])
    cY_cmp = int(M_cmp["m01"] / M_cmp["m00"])
    minDist=Distance_((cX,cY),(cX_cmp,cY_cmp))
    Centroid_a=(cX,cY)
    Centroid_b=(cX_cmp,cY_cmp)
    return minDist,Centroid_a,Centroid_b



def estimate_midlane(midlane_patches, Max_dist):

    # 1. Keep a Midlane_draw for displaying the shortest connectivity later on
    midlane_connectivity_bgr = cv2.cvtColor(midlane_patches, cv2.COLOR_GRAY2BGR)

    #Extract the contours that define each object
    contours = cv2.findContours(midlane_patches, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]

    #Keep only those contours that are not lines
    min_area = 1
    legit_contours = []
    for _,cnt in enumerate(contours):
        cnt_area = cv2.contourArea(cnt)
        if (cnt_area>min_area):
            legit_contours.append(cnt)

    contours = legit_contours

    #Connect each contours with its closest and disconnecting any that are farther than x
    #distance

    CntIdx_BsMatch = []
    for index, cnt in enumerate(contours):
        prevmin_dist = 100000; Bstindex_cmp = 0; BstCentroid_a = 0; BstCentroid_b = 0
        for index_cmp in range(len(contours)-index):
            index_cmp = index_cmp + index
            cnt_cmp = contours[index_cmp]

            #We don't want to compare a contour with itself
            if (index!= index_cmp):
                min_dist, cent_a, cent_b = ApproxDistBWCntrs(cnt, cnt_cmp)
                if (min_dist<prevmin_dist):
                    #check if we have found the closest contour
                    if(len(CntIdx_BsMatch)==0):
                        prevmin_dist = min_dist
                        Bstindex_cmp = index_cmp
                        BstCentroid_a = cent_a
                        BstCentroid_b = cent_b
                    else:
    #When a candidate best match has been found, we loop over the list to see if that match 
    #has been found before we ignore it
                        already_found = False
                        for i in range(len(CntIdx_BsMatch)):
                            if ((index_cmp == i) and (index == CntIdx_BsMatch[i])):
                                already_found = True
                        if not already_found:
                            prevmin_dist = min_dist
                            Bstindex_cmp = index_cmp
                            BstCentroid_a = cent_a
                            BstCentroid_b = cent_b

        if ((prevmin_dist!=100000) and (prevmin_dist>Max_dist)):
            break

        if(type(BstCentroid_a)!=int):
            CntIdx_BsMatch.append(Bstindex_cmp)
            cv2.line(midlane_connectivity_bgr, BstCentroid_a, BstCentroid_b,(0,255,0),2)

    midlane_connectivity = cv2.cvtColor(midlane_connectivity_bgr, cv2.COLOR_BGR2GRAY)

    #Get estimated midlane by returning the largest contour
    estimated_midlane, largest_found = RetLargestContour(midlane_connectivity)

    # Return Estimated Midlane if found otherwise send original

    if largest_found:
        return estimated_midlane
    else:
        return midlane_patches
            
