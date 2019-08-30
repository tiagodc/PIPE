import pandas as pd 
import numpy as np
from collections import defaultdict
from math import *
import operator
import csv



#calcula a distancia minima para realizar o match entre os timestamps
def closest(myList,myNumber):
	return min(myList, key=lambda x:abs(x-myNumber))

def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        v = tuple(n / mag for n in v)
    return v

## formulas do quaterniuo

def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z


def q_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def qv_mult(q1, v1):
    q2 = (0.0,) + v1
    return q_mult(q_mult(q1, q2), q_conjugate(q1))[1:]



## se necessário converter axis para quaternio

def axisangle_to_q(v, theta, bag, tf):
    v = normalize(v)
    x, y, z = v
    theta /= 2
    w = cos(theta)
    x = x * sin(theta)
    y = y * sin(theta)
    z = z * sin(theta)
    return w, x, y, z


def q_to_axisangle(q):
    w, v = q[0], q[1:]
    theta = acos(w) * 2.0
    return normalize(v), theta  


def calculate_over(indextf,pose,bag,tf,d):
    indexes_bag = d[pose]

    for i in indexes_bag:
        vector = (bag['Pointx'][i],bag['Pointy'][i],bag['Pointz'][i])
        q = (tf['rotw'][indextf],tf['rotx'][indextf],tf['roty'][indextf],tf['rotz'][indextf])

        #vector = normalize(vector)

        a = qv_mult(q,vector)

        b = (tf['transx'][indextf],tf['transy'][indextf],tf['transz'][indextf])
 

        v = tuple(map(operator.add, a, b))


        print("antigo")
        print(vector)
        print("novo")
        print(v)
        row = [str(bag['sec'][i]), str(bag['nsec'][i]), str(v[0]), str(v[1]),str(v[2])]
        print("-------------------------------------------------")
        with open('bagTransformed.csv', 'a') as csvFile:
            writer = csv.writer(csvFile)
            writer.writerow(row)
        csvFile.close();







def main():


    csvHeader = ['sec','nsec', 'Pointx', 'Pointy', 'Pointz']

    with open('bagTransformed.csv', 'w') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerow(csvHeader)
    csvFile.close();

    bag = pd.read_csv("ento_u_bag2.csv")
    tf  = pd.read_csv("ento_u_tf.csv")
    print(bag['sec'].dtype)
    print(tf['sec'].dtype)
    print(tf['nsec'].dtype)

    d = defaultdict(list)

    with open('bagfileConverted.csv', 'w') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerow(csvHeader)
    csvFile.close();



    tf_stamp_list = []
    bag_stamp_list = []
    bag_stamp_list.append(-1.0)

    indexList = -1



    #criação dos indices de timestamp para realizar o pareamento mais rapido
    for index, row in tf.iterrows():
    	#print(row['sec'], row['c2'])
    	time = float(row['sec']) + float(row['nsec']) / 10**9
    	tf_stamp_list.append(time)

    for index, row in bag.iterrows():
    	#print(row['sec'], row['c2'])
    	time = float(row['sec']) + float(row['nsec']) / 10**9
    	if(bag_stamp_list[-1] != time):
    		print("diff")
    		print(index)
    		bag_stamp_list.append(time)
    		indexList +=1
    		d[indexList].append(index)
    	else:
    		d[indexList].append(index)

    ##dummy1 = closest(bag_stamp_list,tf_stamp_list[0])
    ##dummy2 = closest(bag_stamp_list,tf_stamp_list[1])
    ##dummy3 = closest(bag_stamp_lit,tf_stamp_list[2]
    ##dummy4 = closest(bag_stamp_list,tf_stamp_list[3])
    ##dummy5 = closest(bag_stamp_list,tf_stamp_list[4])




    #realiza o pareamente e já processa os frames
    length = len(tf_stamp_list)
    for z in range(length):

        dummy = closest(bag_stamp_list,tf_stamp_list[z])
        index_cur_pose = bag_stamp_list.index(dummy) -1
        index_cur_tf = z
        calculate_over(index_cur_tf,index_cur_pose,bag,tf,d)

    ##index_cur_pose= bag_stamp_list.index(dommy1) -1
    ##index_cur_tf = 0

    ##print(index_cur_pose)
    ##print(tf_stamp_list[0])
    ##print(dummy2)
    ##print(tf_stamp_list[1])
    ##print(dummy3)
    ##print(tf_stamp_list[2])
    ##print(dummy4)
    ##print(tf_stamp_list[3])
    ##print(dummy5)
    ##print(tf_stamp_list[4])



    ##print(tf_stamp_list[0])  
if __name__ == "__main__":
    main()
