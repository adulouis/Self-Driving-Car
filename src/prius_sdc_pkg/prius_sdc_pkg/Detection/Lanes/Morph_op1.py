import cv2
import numpy as np
import math
import time
from .utilities import Distance, Distance_
from ...config import config


""" def BwareaOpen(img,MinArea):

    thresh = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY)[1]
    # Filter using contour area and remove small noise
    cnts = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]
    cnts_TooSmall = []
    for index, cnt in enumerate(cnts):
        area = cv2.contourArea(cnt)
        if area < MinArea:
            cnts_TooSmall.append(cnt)
    
    thresh = cv2.drawContours(thresh, cnts_TooSmall, -1, 0, -1) # [ contour = less then minarea contour, contourIDx, Colour , Thickness ]
            
    return thresh """



def BwareaOpen(img, MinArea):
    # Convert image to gray
    thresh = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
    #thresh = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    #thresh = cv2.adaptiveThreshold(thresh, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    #cv2.imshow('thresh', thresh)
    # Find contours
    contours = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    print(f"Number of contours found: {len(contours)}")
    contours = contours[0] if len(contours) == 2 else contours[1]

    if contours is None:
        print("No contours found.")
        return thresh
    
    # Filter using contour area and remove small noise
    cnts_TooSmall = [cnt for cnt in contours if cv2.contourArea(cnt) < MinArea]
    
    # Draw contours that are too small
    for cnt in cnts_TooSmall:
        cv2.drawContours(thresh, [cnt], -1, 0, thickness=cv2.FILLED)
    
    return thresh 



def FindExtremas(img):
    positions = np.nonzero(img) # position[0] 0 = rows 1 = cols
    if (len(positions)!=0):
        top = positions[0].min()
        bottom = positions[0].max()
        left = positions[1].min()
        right = positions[1].max()
        return top,bottom
    else:
        return 0,0

def FindLowestRow(img):
    positions = np.nonzero(img) # position[0] 0 = rows 1 = cols
    
    if (len(positions)!=0):
        top = positions[0].min()
        bottom = positions[0].max()
        left = positions[1].min()
        right = positions[1].max()
        return bottom
    else:
        return img.shape[0]

def RetLargestContour(gray):
    LargestContour_Found = False
    thresh=np.zeros(gray.shape,dtype=gray.dtype)
    _,bin_img = cv2.threshold(gray,0,255,cv2.THRESH_BINARY)
    #Find the two Contours for which you want to find the min distance between them.
    cnts = cv2.findContours(bin_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]
    Max_Cntr_area = 0
    Max_Cntr_idx= -1
    for index, cnt in enumerate(cnts):
        area = cv2.contourArea(cnt)
        if area > Max_Cntr_area:
            Max_Cntr_area = area
            Max_Cntr_idx = index
            LargestContour_Found = True
    if (Max_Cntr_idx!=-1):
        thresh = cv2.drawContours(thresh, cnts, Max_Cntr_idx, (255,255,255), -1) # [ contour = less then minarea contour, contourIDx, Colour , Thickness ]
    return thresh, LargestContour_Found


def RetLargestContour_OuterLane(gray, minArea):
    # Initialize the output variables
    LargestContour_Found = False
    thresh = np.zeros(gray.shape, dtype=gray.dtype)
    
    # Convert image to binary
    _, bin_img = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)
    
    # Perform morphological operations to clean the binary image
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_DILATE, kernel)
    bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_ERODE, kernel)
    
    # Find contours in the binary image
    contours = cv2.findContours(bin_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    
    # Check if any contours are found
    if not contours:
        print("No contours found.")
        return thresh, LargestContour_Found
    
    # Find the largest contour by area
    Max_Cntr_area = 0
    Max_Cntr_idx = -1
    for index, cnt in enumerate(contours):
        if cnt is not None and len(cnt) > 0:
            area = cv2.contourArea(cnt)
            if area > Max_Cntr_area:
                Max_Cntr_area = area
                Max_Cntr_idx = index
                LargestContour_Found = True
    
    # Check if the largest contour meets the minimum area requirement
    if Max_Cntr_area < minArea:
        LargestContour_Found = False
    
    # Draw the largest contour if it meets the criteria
    if LargestContour_Found and Max_Cntr_idx != -1:
        thresh = cv2.drawContours(thresh, contours, Max_Cntr_idx, 255, thickness=cv2.FILLED)
    
    return thresh, LargestContour_Found
"""


def RetLargestContour_OuterLane(gray,minArea):
    LargestContour_Found = False
    thresh=np.zeros(gray.shape,dtype=gray.dtype)
    _,bin_img = cv2.threshold(gray,0,255,cv2.THRESH_BINARY)
    
    #################################### TESTING SHADOW BREAKER CODE BY DILATING####################
    # 3. Dilating Segmented ROI's
    kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(5,5))
    bin_img_dilated = cv2.morphologyEx(bin_img, cv2.MORPH_DILATE, kernel)    #Find the two Contours for which you want to find the min distance between them.
    bin_img_ret = cv2.morphologyEx(bin_img_dilated, cv2.MORPH_ERODE, kernel)    #Find the two Contours for which you want to find the min distance between them.
    bin_img = bin_img_ret
    #################################### TESTING SHADOW BREAKER CODE BY DILATING####################

    cnts = cv2.findContours(bin_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]
    Max_Cntr_area = 0
    Max_Cntr_idx= -1
    for index, cnt in enumerate(cnts):
        area = cv2.contourArea(cnt)
        if area > Max_Cntr_area:
            Max_Cntr_area = area
            Max_Cntr_idx = index
            LargestContour_Found = True
    
    if Max_Cntr_area < minArea:
        LargestContour_Found = False
    if ((Max_Cntr_idx!=-1) and (LargestContour_Found)):
        thresh = cv2.drawContours(thresh, cnts, Max_Cntr_idx, (255,255,255), -1) # [ contour = less then minarea contour, contourIDx, Colour , Thickness ]
    return thresh, LargestContour_Found

"""


def ROI_extracter(image,strtPnt,endPnt):
    #  Selecting Only ROI from Image
    ROI_mask = np.zeros(image.shape, dtype=np.uint8)
    cv2.rectangle(ROI_mask,strtPnt,endPnt,255,thickness=-1)
    #image_ROI = cv2.bitwise_and(image,image,mask=ROI_mask)
    image_ROI = cv2.bitwise_and(image,ROI_mask)
    return image_ROI

def ExtractPoint(img,specified_row):
    Point= (0,specified_row)
    specified_row_data = img[ specified_row-1,:]
    #print("specified_row_data",specified_row_data)
    positions = np.nonzero(specified_row_data) # position[0] 0 = rows 1 = cols
    #print("positions",positions)    
    #print("len(positions[0])",len(positions[0]))    
    if (len(positions[0])!=0):
        #print(positions[0])
        min_col = positions[0].min()
        Point=(min_col,specified_row)
    return Point


"""
def Ret_LowestEdgePoints(gray):
    
    Outer_Points_list=[]
    thresh = np.zeros(gray.shape,dtype=gray.dtype)
    Lane_OneSide=np.zeros(gray.shape,dtype=gray.dtype)
    Lane_TwoSide=np.zeros(gray.shape,dtype=gray.dtype)

    _,bin_img = cv2.threshold(gray,0,255,cv2.THRESH_BINARY)
        #Find the two Contours for which you want to find the min distance between them.
    cnts = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[1]
    thresh = cv2.drawContours(thresh, cnts, 0, (255,255,255), 1) # [ contour = less then minarea contour, contourIDx, Colour , Thickness ]
    # Boundary of the Contour is extracted and Saved in Thresh

    Top_Row,Bot_Row = FindExtremas(thresh)
    cv2.imshow("thresh",thresh)

    Contour_TopBot_PortionCut = ROI_extracter(thresh,(0, Top_Row + 5),(thresh.shape[1],Bot_Row-5))

    cnts2 = cv2.findContours(Contour_TopBot_PortionCut, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[1]

    LowRow_a=-1
    LowRow_b=-1
    
    Euc_row=0# Row for the points to be compared

    First_line = np.copy(Lane_OneSide)
    cnts_tmp = []


    if(len(cnts2)>1):
        for index_tmp, cnt_tmp in enumerate(cnts2):
            if((cnt_tmp.shape[0])>50):
                cnts_tmp.append(cnt_tmp)
        cnts2 = cnts_tmp

    for index, cnt in enumerate(cnts2):
        Lane_OneSide = np.zeros(gray.shape,dtype=gray.dtype)
        Lane_OneSide = cv2.drawContours(Lane_OneSide, cnts2, index, (255,255,255), 1) # [ contour = less then minarea contour, contourIDx, Colour , Thickness ]
        Lane_TwoSide = cv2.drawContours(Lane_TwoSide, cnts2, index, (255,255,255), 1) # [ contour = less then minarea contour, contourIDx, Colour , Thickness ]

        if(len(cnts2)==2):
            if (index==0):
                First_line = np.copy(Lane_OneSide)
                LowRow_a = FindLowestRow(Lane_OneSide)
            elif(index==1):
                LowRow_b = FindLowestRow(Lane_OneSide)
                if(LowRow_a<LowRow_b):# First index is shorter 
                    Euc_row=LowRow_a
                else:
                    Euc_row=LowRow_b
                Point_a = ExtractPoint(First_line,Euc_row)
                Point_b = ExtractPoint(Lane_OneSide,Euc_row)
                Outer_Points_list.append(Point_a)
                Outer_Points_list.append(Point_b)
    
    return Lane_TwoSide, Outer_Points_list

"""


def Ret_LowestEdgePoints(gray):
    Outer_Points_list = []
    thresh = np.zeros(gray.shape, dtype=gray.dtype)
    Lane_OneSide = np.zeros(gray.shape, dtype=gray.dtype)
    Lane_TwoSide = np.zeros(gray.shape, dtype=gray.dtype)

    _, bin_img = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)

    # Find contours in the binary image
    contours = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours = contours[0] if len(contours) == 2 else contours[1]

    # Check if any contours are found
    if not contours:
        print("No contours found.")
        return Lane_TwoSide, Outer_Points_list

    # Draw the first contour (if it exists) onto the threshold image
    if len(contours) > 0 and len(contours[0]) > 0:
        thresh = cv2.drawContours(thresh, contours, 0, (255, 255, 255), 1)

    # Find the top and bottom row of the contours
    Top_Row, Bot_Row = FindExtremas(thresh)
    #cv2.imshow("thresh", thresh)

    # Extract the region of interest from the threshold image
    Contour_TopBot_PortionCut = ROI_extracter(thresh, (0, Top_Row + 5), (thresh.shape[1], Bot_Row - 5))

    # Find contours in the extracted region
    contours2 = cv2.findContours(Contour_TopBot_PortionCut, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours2 = contours2[0] if len(contours2) == 2 else contours2[1]

    # Filter contours that have less than 50 points
    filtered_contours = [cnt for cnt in contours2 if cnt is not None and len(cnt) > 50]

    # Initialize variables for finding the lowest row points
    LowRow_a = -1
    LowRow_b = -1
    Euc_row = 0
    First_line = np.copy(Lane_OneSide)

    # Process each contour
    for index, cnt in enumerate(filtered_contours):
        if cnt is not None and len(cnt) > 0:
            Lane_OneSide = np.zeros(gray.shape, dtype=gray.dtype)
            Lane_OneSide = cv2.drawContours(Lane_OneSide, filtered_contours, index, (255, 255, 255), 1)
            Lane_TwoSide = cv2.drawContours(Lane_TwoSide, filtered_contours, index, (255, 255, 255), 1)

            if len(filtered_contours) == 2:
                if index == 0:
                    First_line = np.copy(Lane_OneSide)
                    LowRow_a = FindLowestRow(Lane_OneSide)
                elif index == 1:
                    LowRow_b = FindLowestRow(Lane_OneSide)
                    if LowRow_a < LowRow_b:
                        Euc_row = LowRow_a
                    else:
                        Euc_row = LowRow_b
                    Point_a = ExtractPoint(First_line, Euc_row)
                    Point_b = ExtractPoint(Lane_OneSide, Euc_row)
                    if Point_a is not None and Point_b is not None:
                        Outer_Points_list.append(Point_a)
                        Outer_Points_list.append(Point_b)

    return Lane_TwoSide, Outer_Points_list


def ApproxDistBWCntrs(cnt,cnt_cmp):
    # compute the center of the contour
    M = cv2.moments(cnt)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    # compute the center of the contour
    M_cmp = cv2.moments(cnt_cmp)
    cX_cmp = int(M_cmp["m10"] / M_cmp["m00"])
    cY_cmp = int(M_cmp["m01"] / M_cmp["m00"])
    minDist=Distance_((cX,cY),(cX_cmp,cY_cmp))
    Centroid_a=(cX,cY)
    Centroid_b=(cX_cmp,cY_cmp)
    return minDist,Centroid_a,Centroid_b

def Estimate_MidLane(BW,MaxDistance):
    #cv2.namedWindow("BW_zero",cv2.WINDOW_NORMAL)
    BW_zero= cv2.cvtColor(BW,cv2.COLOR_GRAY2BGR)
    #Find the two Contours for which you want to find the min distance between them.
    cnts= cv2.findContours(BW, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[1]#3ms
    MinArea=1
    cnts_Legit=[]
    for index, _ in enumerate(cnts):
        area = cv2.contourArea(cnts[index])
        if area > MinArea:
            cnts_Legit.append(cnts[index])
    cnts=cnts_Legit
    # Cycle through each point in the Two contours & find the distance between them.
    # Take the minimum Distance by comparing all other distances & Mark that Points.
    CntIdx_BstMatch = []# [BstMatchwithCnt0,BstMatchwithCnt1,....]
    Closests_Pixels_list = []
    #200msec
    for index, cnt in enumerate(cnts):
        prevmin_dist = 100000
        Bstindex_cmp = 0
        #BstClosests_Pixels =0 
        BstCentroid_a=0      
        BstCentroid_b=0      
        for index_cmp in range(len(cnts)-index):
            index_cmp = index_cmp + index
            cnt_cmp = cnts[index_cmp]
            if (index!=index_cmp):
                min_dist,Centroid_a,Centroid_b  = ApproxDistBWCntrs(cnt,cnt_cmp)

                #Closests_Pixels=(cnt[min_dstPix_Idx[0]],cnt_cmp[min_dstPix_Idx[1]])
                if(min_dist < prevmin_dist):
                    if (len(CntIdx_BstMatch)==0):
                        prevmin_dist = min_dist
                        Bstindex_cmp = index_cmp
                        #BstClosests_Pixels = Closests_Pixels
                        BstCentroid_a=Centroid_a
                        BstCentroid_b=Centroid_b   

                    else:
                        Present= False
                        for i in range(len(CntIdx_BstMatch)):
                            if ( (index_cmp == i) and (index == CntIdx_BstMatch[i]) ):
                                Present= True
                        if not Present:
                            prevmin_dist = min_dist
                            Bstindex_cmp = index_cmp
                            #BstClosests_Pixels = Closests_Pixels
                            BstCentroid_a=Centroid_a
                            BstCentroid_b=Centroid_b   
        if ((prevmin_dist!=100000 ) and (prevmin_dist>MaxDistance)):
            break
        if (type(BstCentroid_a)!=int):
            CntIdx_BstMatch.append(Bstindex_cmp)
            #Closests_Pixels_list.append(BstClosests_Pixels)
            #cv2.line(BW_zero,(BstClosests_Pixels[0][0][0],BstClosests_Pixels[0][0][1]),(BstClosests_Pixels[1][0][0],BstClosests_Pixels[1][0][1]),(0,0,255),thickness=2)
            cv2.line(BW_zero,BstCentroid_a,BstCentroid_b,(0,255,0),thickness=2)
            #cv2.imshow("BW_zero",BW_zero)
    
    #cv2.imwrite("D:/Had_LuQ/MidlaneClosestJoined.png",BW_zero)
    BW_zero = cv2.cvtColor(BW_zero,cv2.COLOR_BGR2GRAY)

    BW_Largest,Largest_found = RetLargestContour(BW_zero)#3msec

    if(Largest_found):
        return BW_Largest
    else:
        return BW