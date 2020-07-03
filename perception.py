import numpy as np
import cv2 as cv

## execise 1-1
# segment object using color
def _segment_redball(rgb):
    rgb = cv.cvtColor(rgb, cv.COLOR_BGR2RGB)
    hsv = cv.cvtColor(rgb, cv.COLOR_BGR2HSV)

    mask1 = cv.inRange(hsv, (  0, 120, 70), ( 10, 255, 255))
    mask2 = cv.inRange(hsv, (170, 120, 70), (180, 255, 255))
    mask = mask1 + mask2
    filtered_mask = np.zeros(mask.shape, np.uint8)
    contours, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

    if len(contours)>0:        
        # find largest contour
        largest, idx = 0., None
        for i, c in enumerate(contours):
            # remove noise
            if c.shape[0]<10: continue
            if cv.contourArea(c)>largest: 
                largest = cv.contourArea(c)
                idx = i
    
        cv.drawContours(filtered_mask, contours[idx], -1, (255,255,255),-1)    
    return filtered_mask


## execise 1-2
def _image_pointcloud(depth, mask):
    mask_pixels = np.where(mask>0)
    pointcloud = np.empty((mask_pixels[0].shape[0], 3))
    pointcloud[:,0] = mask_pixels[1]  # x pixels
    pointcloud[:,1] = mask_pixels[0]  # y pixels
    pointcloud[:,2] = depth[mask_pixels[0], mask_pixels[1]]
    return pointcloud


## execise 1-3
def _meter_pointcloud(pixel_points, fxfypxpy):
    points = np.empty(np.shape(pixel_points))
    for i, p in enumerate(pixel_points):
        x = p[0]
        y = p[1]
        d = p[2]
        
        px = fxfypxpy[-2]
        py = fxfypxpy[-1]
        
        x_ =  d * (x-px) / fxfypxpy[0]
        y_ = -d * (y-py) / fxfypxpy[1]
        z_ = -d
        points[i] = [x_,y_,z_]
    return points


def find_ball(rgb, depth, fxfypxpy):
    mask = _segment_redball(rgb)
    pixel_points = _image_pointcloud(depth, mask)
    obj_points = _meter_pointcloud(pixel_points, fxfypxpy)
    return obj_points, mask


###########################################################

# def _segment_depth(depth, depth0):
#     depth_mask = cv.absdiff(depth, depth0)
#     ret, mask = cv.threshold(depth_mask, 0, 5, cv.THRESH_BINARY)
#     mask = mask * 255
#     return mask.astype("uint8")


# # combining both color & depth, requires initial depth image
# def segment_object(rgb, depth, depth0):
#     mask_rgb = _segment_redball(rgb)
#     mask_depth = _segment_depth(depth, depth0)
#     mask = cv.bitwise_and(mask_rgb, mask_depth)
#     return mask