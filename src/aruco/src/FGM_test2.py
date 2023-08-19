#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import math as m

def linear_interpolate(array):
    array = list(array)
    for i in range (0, len(array)):
        if array[i] == 0:
            count = 0
            value = array[i]
            while value == 0:
                count +=1
                i+=1
                if i >=len(array):
                    diff = array[i-count-1] - array[i-count-2]
                    for j in range(i-count, i):
                        array[j] = array[j-1] + diff
                    return array
                value = array[i]
            diff = array[i+1] - array[i]
            for j in range(i-1, i-count-1, -1):
                array[j] = array[j+1] - diff
    return array

def depth_sort(array):
    array = list(array)
    for i in range (0, len(array)):
        if not (array[i]>0 and array[i] < 5.0):
            array[i] = 0
    return array

def gap_angle(angle_array, depth_array):
    angle_array = list(angle_array)
    depth_array = list(depth_array)
    gap_left = [None]*len(angle_array)
    gap_right = [None]*len(angle_array)
    left_gap = [None]*len(angle_array)
    right_gap = [None]*len(angle_array)
    for i in range(len(angle_array)):
        if depth_array[i] !=0:
            angle = m.asin(0.05/depth_array[i])
            gap_left[i]= angle_array[i] - angle
            gap_right[i] = angle_array[i] + angle
    
    i=0
    while i < (len(angle_array)-1):
        if gap_left[i] !=None and gap_right[i] != None:
            left_gap[i] = gap_left[i]
            k=i
            right__gap = gap_right[i]
            while (gap_left[i+1] != None) and (gap_left[i+1] > (gap_left[i]-1/180*m.pi)) and (gap_left[i+1] < (gap_right[i] + 1/180*m.pi)):
                i+=1
                right__gap = gap_right[i]
                if i >= len(angle_array)-1:
                    break
            right_gap[k] = right__gap
        i+=1
    return left_gap , right_gap

def insertion_sort(list1,list2): 
        for i in range(1, len(list1)): 
            a = list1[i] 
            b = list2[i]
            j = i - 1 
            while j >= 0 and a < list1[j]: 
                list1[j + 1] = list1[j] 
                list2[j+1] = list2[j]
                j -= 1 
            list1[j + 1] = a 
            list2[j+1] = b
        return list1,list2

depth_array = []
def callback1(array):
    global depth_array
    depth_array = array.data

angle_array = []
def callback2(array):
    global angle_array
    angle_array = array.data
    global depth_array
    angle_array = linear_interpolate(angle_array)
    print(angle_array)
    depth_array = depth_sort(depth_array)
    left_angle, right_angle = gap_angle(angle_array, depth_array)
    gap_array_left = []
    gap_array_right = []
    for i in range(len(left_angle)):
        if left_angle[i] != None:
            gap_array_left.append(left_angle[i])
            gap_array_right.append(right_angle[i])
    # gap_array_left,gap_array_right = insertion_sort(gap_array_left,gap_array_right)
    gap_array_left,gap_array_right = insertion_sort(gap_array_left,gap_array_right)
    i=-1
    while i <(len(gap_array_left)-1):
        i+=1
        # left = gap_array_left[i]
        right = gap_array_right[i]
        j=i
        while j<len(gap_array_left)-1:
            j+=1
            if gap_array_left[j] < right and  gap_array_right[j] < right:
                gap_array_left[j]=None
                gap_array_right[j]=None
            elif gap_array_left[j] < right and gap_array_right[j] > right:
                gap_array_right[i] = gap_array_right[j]
                gap_array_left[j]=None
                gap_array_right[j]=None
    left=[]
    right=[]
    for i in range(len(gap_array_left)):
        if gap_array_left[i] !=None:
            left.append(gap_array_left[i])
            right.append(gap_array_right[i])
    print(left,right)
        

if __name__ == '__main__':
    rospy.init_node('FGM_max_gap_calc')
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rospy.Subscriber('/FGM_DEPTH_ARRAY', Float64MultiArray, callback1)
        rospy.Subscriber('/FGM_ANGLE_ARRAY', Float64MultiArray, callback2)
        rate.sleep()
    