# -*- coding: utf-8 -*-

# Commented out IPython magic to ensure Python compatibility.
# %pip install amplpy

import math
import json
import random
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import sys
import re
from sympy.geometry import Line
from sympy import Point
from sympy import Point2D

# input parameters
topology = sys.argv[1]  # 1 grid, 2 airport, 3 metroplex
used_seed = sys.argv[2]
input_file = ""
if topology == "1":
	input_file = "/home/magi/UAMdeconflictionMasterThesis/modelli/data/mercedesSD/grid" + used_seed + ".txt"
elif topology == "2":
	input_file = "/home/magi/UAMdeconflictionMasterThesis/modelli/data/mercedesSD/airport" + used_seed + ".txt"
elif topology == "3":
	input_file = "/home/magi/UAMdeconflictionMasterThesis/modelli/data/mercedesSD/metroplex" + used_seed + ".txt"
output_file = "/home/magi/UAMdeconflictionMasterThesis/modelli/data/mercedesTD/"
random_seed = used_seed
verbose = int("0")  # print outputs?
compareWithLocal = int(
    "0"
)  # run AMPL only with "inConflict" pairs (not global approach)
nDrift = int(sys.argv[3])  # scenario 1: num drift
#TODO  provare con 1 drift e basta, con 1,2,3 delays senza drift e 1 drift e 1 delay
nDelay = int(sys.argv[4])  # scenario 1: num delayed
nIntru = int(
    "0"
)  # scenario 2: if nIntru>0 then collaborative intruder. max time ahead before intruder's entrance < nIntru --> 1
nNonColIntru = int(
    "0"
)  # scenario 3: if nNonColIntru>0 then non-collaborative intruder.   --> 1
max_acceler = float("5")  # % max acceleration, affects q_max  + 5, -5
max_deceler = float("5")  # % max deceleration, affects q_min  + 5, -5

if topology == "1":
	output_file += "grid" + used_seed +"nDr"+ str(nDrift)+"nDe"+str(nDelay)+".dat"
elif topology == "2":
	output_file += "airport" + used_seed +"nDr"+ str(nDrift)+"nDe"+str(nDelay)+".dat"
elif topology == "3":
	output_file += "metroplex" + used_seed +"nDr"+ str(nDrift)+"nDe"+str(nDelay)+".dat"
random.seed(random_seed)
# parameters from SD
D = 0
v_default = 0
v_climb = 0
numBatches = 0
tripsPerBatch = 0
timeBatch = 0
nNodes = 0
nEdges = 0
nFatos = 0
coord_x = {}
coord_y = {}
nodeType = (
    {}
)  # 0=take off junc (insertion); 1=landing junc (split); 2= inner junc (sync); 3=take off FATO; 4=landing FATO
isFATO = {}
takeoffFATO_list = []
landingFATO_list = []
tuples_nodePath = []
tuples_linkPath = []
orig_list = {}
dest_list = {}
edges = []
skylanes = []
dist = {}
angleJunc = {}
tplanMin = {}
tplanMax = {}
schedule = json.loads("""{"trips":[]}""")  # json list with trips
latestArrival_nominal = 0
climb_slackSD = 0
cruise_slackSD = 0
# deconf parameters:
qmin_default = (100 - max_deceler) / 100
qmax_default = (100 + max_acceler) / 100
aheadMaxDef = 2
delayMaxDef = 2
vmin = 0
vmax = 0
# disruptions parameters:
epsilonDelay = 0.1
epsilonDrift = 0.5
min_trip_duration = 7  # for intruder's path
max_trip_duration = 25
# program parameters
G = nx.DiGraph()
tErr = 1

def read_SD_schedule(fileName):
    global D
    global v_default
    global v_climb
    global numBatches
    global tripsPerBatch
    global timeBatchnum
    global nNodes
    global nEdges
    global nFatos
    global latestArrival_nominal
    global climb_slackSD
    global cruise_slackSD
    global vmin
    global vmax
    file = open(fileName)
    line = file.readline()  # nNodes
    line_split = re.split("[:]", line)
    nNodes = int(line_split[1])
    line = file.readline()  # nFatos
    line_split = re.split("[:]", line)
    nFatos = int(line_split[1])
    line = file.readline()  # node list
    # defines the nodes
    for i in range(nNodes):
        line = file.readline()
        entry = line.split()
        node = int(entry[0])
        code = int(entry[1])
        coord_x[node] = float(entry[2])
        coord_y[node] = float(entry[3])
        nodeType[node] = code
        if code == 3 or code == 4:
            isFATO[node] = 1
            if code == 3:
                takeoffFATO_list.append(node)
            else:
                landingFATO_list.append(node)
        else:
            isFATO[node] = 0
    line = file.readline()  # nEdges
    line_split = re.split("[:]", line)
    nEdges = int(line_split[1])
    line = file.readline()  # edge list
    # define the edges
    for i in range(nEdges):
        line = file.readline()
        entry = line.split()
        u = int(entry[0])
        v = int(entry[1])
        dist[u, v] = float(entry[2])
        edges.append((u, v))
        G.add_edge(u, v, dist=dist[u, v])
    line = file.readline()  # nSkylanes
    line_split = re.split("[:]", line)
    nSkylanes = int(line_split[1])
    line = file.readline()  # skylanes list
    # define skylanes --> a cosa mi serve?
    for i in range(nSkylanes):
        line = file.readline()
        entry = line.split()
        skylanes.append(
            (float(entry[0]), float(entry[1]), float(entry[2]), float(entry[3]))
        )
    line = file.readline()  # nAngles
    line_split = re.split("[:]", line)
    nAngles = int(line_split[1])
    line = file.readline()  # angleJunction list
    # defines the angles
    for i in range(nAngles):
        line = file.readline()
        entry = line.split()
        angleJunc[int(entry[0]), int(entry[1]), int(entry[2])] = float(entry[3])
    line = file.readline()  # D
    line_split = re.split("[:]", line)
    D = float(line_split[1])
    line = file.readline()  # v_default
    line_split = re.split("[:]", line)
    v_default = float(line_split[1])
    line = file.readline()  # v_climb
    line_split = re.split("[:]", line)
    v_climb = float(line_split[1])
    line = file.readline()  # numBatches
    line_split = re.split("[:]", line)
    numBatches = int(line_split[1])
    line = file.readline()  # tripsPerBatch
    line_split = re.split("[:]", line)
    tripsPerBatch = int(line_split[1])
    line = file.readline()  # timeBatch
    line_split = re.split("[:]", line)
    timeBatch = float(line_split[1])
    line = file.readline()  # nTrips
    line_split = re.split("[:]", line)
    nTrips = int(line_split[1])
    for i in range(nTrips):
        line=file.readline() # trip
        entry=line.split()
        uid=int(entry[1])
        line=file.readline() # batch
        line_split=re.split('[:]', line)
        batch=int(line_split[1])
        line=file.readline() # waypoints
        line_split=re.split('[:]', line)
        path=list(map(int,line_split[1].split(',')))
        line=file.readline() # t_ear
        line_split=re.split('[:]', line)
        t_ear=list(map(float,line_split[1].split(',')))
        line=file.readline() # t_lat
        line_split=re.split('[:]', line)
        t_lat=list(map(float,line_split[1].split(',')))
        schedule['trips'].append({'uid':  uid, "batch": batch,"waypoints" : path,"tEarliest" : t_ear,"tLatest" : t_lat})
        for wp in path:
          tuples_nodePath.append((uid,wp))
        for i in range(len(path)-1):
          wp=int(path[i])
          tuples_linkPath.append((uid,wp,int(path[i+1])))
          tplanMin[uid,wp]=float(t_ear[i])
          tplanMax[uid,wp]=float(t_lat[i])
        wp_last=int(path[len(path)-1])
        tplanMin[uid,wp_last]=float(t_ear[len(t_ear)-1])
        tplanMax[uid,wp_last]=float(t_lat[len(t_lat)-1])
        orig_list[uid]=int(path[0])
        dest_list[uid]=wp_last
        if tplanMax[uid,wp_last]> latestArrival_nominal:
          latestArrival_nominal=tplanMax[uid,wp_last]
    line=file.readline() # climb slack
    line_split=re.split('[:]', line)
    climb_slackSD=float(line_split[1])
    line=file.readline() # cruise slack
    line_split=re.split('[:]', line)
    cruise_slackSD=float(line_split[1])
    vmin=v_default*qmin_default
    vmax=v_default*qmax_default


def disruptions_scenario1(nDrift, nDelay, endHorizon):
    global A
    global inflight
    global ground
    global drifted
    global delayed
    global timeTD
    global wpDrift
    global epsilonDelay
    global epsilonDrift
    global timeDrifted
    # trips
    for trip in schedule["trips"]:
        A.append(trip["uid"])
    # create disruptions
    if nDelay > 0:  # there are delays
        if nDelay == 1 and nDrift == 0:
            delayed = random.sample(A, 1)
            uid = delayed[0]
            timeTD = tplanMax[uid, orig_list[uid]] + epsilonDelay
        else:
            # per ogni t,contiene tutti i voli che partono al tempo t
            plannedByT = (
                {}
            )  # dict, plannedByT[t]=[3,6] if trips 3 and 6 have departure planned in [t-1,t]
            for t in range(1, endHorizon - 3):
                for trip in schedule["trips"]:
                    if trip["tLatest"][0] == t:
                        if t not in plannedByT:
                            plannedByT[t] = []
                        plannedByT[t].append(trip["uid"])
            # check that nDelay<=  max{ len(plannedByT[t]) }

            # max t trovato
            maxLen_plannedByT = 0
            for t in range(1, endHorizon - 3):
                if t in plannedByT and len(plannedByT[t]) > maxLen_plannedByT:
                    maxLen_plannedByT = len(plannedByT[t])
            if maxLen_plannedByT < nDelay:
                print(
                    "WARNING: unfeasible number of delays (greater than simultaneous departures)"
                )
                print("\t setting delays to %i" % maxLen_plannedByT)
                nDelay = maxLen_plannedByT
            # remove from plannedByT those lists with less trips than nDelay
            for t in range(1, endHorizon - 3):
                if t in plannedByT and len(plannedByT[t]) < nDelay:
                    plannedByT.pop(t)
            # if there is also a drift, delay cannot happen at the beginning of time horizon
            if nDrift > 0:
                for t in range(1, endHorizon // 5):
                    if t in plannedByT:
                        plannedByT.pop(t)
            tDelay = random.choice([t for t in plannedByT])
            delayed = random.sample(plannedByT[tDelay], nDelay)
            timeTD = tDelay + epsilonDelay
    if nDrift > 0:  # precondition: if this happens we have 1 drift (and 0 or 1 delay)
        drifted = random.sample(list(set(A) - set(delayed)), nDrift)
        if nDelay > 0:  # make sure drift and delay match
            notFound = True
            while notFound:
                uid = drifted[0]
                if (
                    tplanMin[uid, orig_list[uid]] >= timeTD
                    or tplanMin[uid, dest_list[uid]] <= timeTD
                ):
                    # this trip is on ground, cannot be drifted. Chose another one:
                    drifted = random.sample(list(set(A) - set(delayed)), 1)
                else:
                    for trip_drift in schedule["trips"]:
                        if trip_drift["uid"] == uid:
                            for wp_index in range(len(trip_drift["waypoints"])):
                                if trip_drift["tEarliest"][wp_index] >= timeTD:
                                    notFound = False
                                    wpDrift = trip_drift["waypoints"][wp_index]
                                    break
                            break
        else:  # nDrift=1 and nDelay=0
            for trip_drift in schedule["trips"]:
                if trip_drift["uid"] == drifted[0]:
                    wpDrift_index = random.randint(2, len(trip_drift["waypoints"]) - 2)
                    wpDrift = trip_drift["waypoints"][wpDrift_index]
                    timeTD = trip_drift["tEarliest"][wpDrift_index] - epsilonDrift
                    break
        uid = drifted[0]
        rand = random.uniform(0, 1)  # ahead or delay?
        if rand < 0.5:
            tminNew = random.uniform(
                timeTD + epsilonDrift / 2, tplanMin[uid, wpDrift] - 0.01
            )  # ahead
        else:
            tminNew = random.uniform(
                tplanMin[uid, wpDrift] + 0.01, tplanMin[uid, wpDrift] + epsilonDrift / 2
            )  # delay
        tminNew = math.ceil(tminNew * 100) / 100
        timeDrifted = (
            tminNew - tplanMin[uid, wpDrift]
        )  # if negative ahead, otherwise delay
        print(tminNew, tplanMin[uid, wpDrift], timeDrifted)
    # classify trips
    for trip in schedule["trips"]:
        if trip["tEarliest"][0] >= timeTD or trip["uid"] in delayed:
            ground.append(trip["uid"])
        elif trip["tLatest"][len(trip["waypoints"]) - 1] <= timeTD:
            A.remove(trip["uid"])
        else:
            inflight.append(trip["uid"])


def intruder_scenario2(endHorizon):
	global A
	global AP
	global inflight
	global ground
	global timeTD
	global o_intru
	global d_intru
	# trips
	intru_uid=0
	for trip in schedule['trips']:
		A.append(trip['uid'])
		if trip['uid']> intru_uid:
			intru_uid=trip['uid']
	intru_uid+=1
	# Intruder's trajectory
	nSkyports=nFatos/2
	orig_index=random.randint(0,nSkyports-1)
	o_intru=takeoffFATO_list[orig_index]
	d_intru=o_intru
	#creo nuovo volo in maniera casuale
	path=[]
	trip_duration=max_trip_duration+1
	while d_intru== o_intru or trip_duration< min_trip_duration or trip_duration >max_trip_duration:
		dest_index = random.randint(0,nSkyports-1)
		if dest_index != orig_index:
			d_intru=landingFATO_list[dest_index]
			path=nx.shortest_path(G,o_intru, d_intru, weight='dist') # network weight is the distance
			path_distance=nx.shortest_path_length(G,o_intru, d_intru, weight='dist')
			trip_duration=path_distance/v_default
		else:
			d_intru=o_intru
	# Intruder's time schedule
	timeTD=random.uniform(1, endHorizon/5)
	orig_list[intru_uid]=o_intru
	dest_list[intru_uid]=d_intru
	#probabilmente numero waypoint/ numero nodi, definisco t_ear e t_lat attraverso tempo iniziale e path
	numWP=len(path)
	t_ear=[0]*numWP
	t_lat=[0]*numWP
	t_ear[0]=math.ceil(timeTD)
	t_lat[0]=t_ear[0]+tErr
	t_ear[1]=t_ear[0]+G[path[0]][path[1]]['dist']/v_climb
	t_lat[1]=t_lat[0]+G[path[0]][path[1]]['dist']/v_climb
	for i in range(2,numWP-1):
		t_ear[i]=t_ear[i-1]+G[path[i-1]][path[i]]['dist']/v_default
		t_lat[i]=t_lat[i-1]+G[path[i-1]][path[i]]['dist']/v_default
	t_ear[numWP-1]=t_ear[numWP-2]+G[path[numWP-2]][path[numWP-1]]['dist']/v_climb
	t_lat[numWP-1]=t_lat[numWP-2]+G[path[numWP-2]][path[numWP-1]]['dist']/v_climb
	#aggiungo il volo ai trips
	schedule['trips'].append({'uid':  intru_uid, "batch": -1,"waypoints" : path,"tEarliest" : t_ear,"tLatest" : t_lat})
	for wp_index in range(numWP):
		wp=path[wp_index]
		tuples_nodePath.append((intru_uid,wp))
		tplanMin[intru_uid,wp]=t_ear[wp_index]
		tplanMax[intru_uid,wp]=t_lat[wp_index]
	for i in range(len(path)-1):
		tuples_linkPath.append((intru_uid,path[i],path[i+1]))
	# Classify trips
	AP=[intru_uid]
	A.append(intru_uid)
	for trip in schedule['trips']:
		if trip['tEarliest'][0]>= timeTD or trip['uid']==intru_uid:
			ground.append(trip['uid'])
		elif trip['tLatest'][len(trip['waypoints'])-1]<= timeTD:
			A.remove(trip['uid'])
		else:
			inflight.append(trip['uid'])

def crossing_angle(m,h,l): # auxiliary function that returns the crossing angle of edges (m,h) and (m,l)
	vx_mh=coord_x[m]-coord_x[h]
	vy_mh=coord_y[m]-coord_y[h]
	vx_ml=coord_x[m]-coord_x[l]
	vy_ml=coord_y[m]-coord_y[l]
	v_mh=[vx_mh,vy_mh]
	v_ml=[vx_ml,vy_ml]
	unit_v_mh = v_mh/ np.linalg.norm(v_mh)
	unit_v_ml = v_ml/ np.linalg.norm(v_ml)
	dot_product = np.dot(unit_v_mh, unit_v_ml)
	angle = np.arccos(dot_product)
	return angle
#------------ end function
def draw_skylane():  # auxiliary function to draw skylane network
	coord_planar={}
	coord_junc={}
	coord_ports={}
	color_ports=[]
	ports_list=[]
	junc_list=[]
	label_map={}
	for u in range(nNodes):
		if nodeType[u]==3 or nodeType[u]==4: # node is skyport
			coord_ports[u]=[coord_x[u],coord_y[u]+0.5]
			coord_planar[u]=coord_ports[u]
			ports_list.append(u)
			color_ports.append("gray")
			label_map[u]=u
		elif nodeType[u]==0 or nodeType[u]==1: # node is skyport junction
			coord_ports[u]=[coord_x[u],coord_y[u]]
			coord_planar[u]=coord_ports[u]
			ports_list.append(u)
			color_ports.append("white")
			label_map[u]=u
		else: # node is inner junction
			coord_junc[u]=[coord_x[u],coord_y[u]]
			coord_planar[u]=coord_junc[u]
			junc_list.append(u)
			label_map[u]=u

	nx.draw_networkx_nodes(G, pos=coord_ports, node_size=350,node_shape="d",nodelist=ports_list,node_color=color_ports)
	nodes=nx.draw_networkx_nodes(G, pos=coord_junc, node_size=350,node_shape="o",node_color="white",nodelist=junc_list)
	if nodes:
		nodes.set_edgecolor('black')
	nx.draw_networkx_edges(G,coord_planar,arrows=True)
	nx.draw_networkx_labels(G,coord_planar, labels=label_map)

	ax= plt.gca()
	ax.collections[0].set_edgecolor("#000000")

	plt.axis('off')
	plt.show()
#------------ end function
def intruder_scenario3(endHorizon):
	global A
	global AP_NC
	global inflight
	global ground
	global timeTD
	global nNodes
	global schedule
	global G
	# Save skylanes as Line objects
	list_skylanes=[]
	#skylanes è una variabile "globale"
	nSkylanes=len(skylanes)
	for i in range(nSkylanes):
		l = Line(Point(skylanes[i][0],skylanes[i][1]), Point(skylanes[i][2],skylanes[i][3]))
		list_skylanes.append(l)
	# Save trips in A and set intruder uid
	intru_uid=0
	for trip in schedule['trips']:
		A.append(trip['uid'])
		if trip['uid']> intru_uid:
			intru_uid=trip['uid']
	intru_uid+=1
	# ------- Intruder's trajectory
	# generate randomly a waypoint at cruising level
	cruise_nodes=list(set(range(nNodes))-set(landingFATO_list)-set(takeoffFATO_list))
	nCruise=len(cruise_nodes)
	orig_index=random.randint(0,nCruise-1)
	wp_0=cruise_nodes[orig_index]
	# generate randomly another waypoint at cruising level
	dest_index=random.randint(0,nCruise-1)
	wp_k=cruise_nodes[dest_index]
	# this 2nd wp cannot be aligned with the 1st one (both on the same skylane)
	colinear=False
	p0=Point(coord_x[wp_0],coord_y[wp_0])
	pk=Point(coord_x[wp_k],coord_y[wp_k])
	for i in range(nSkylanes):
		if list_skylanes[i].contains(p0) and list_skylanes[i].contains(pk): colinear=True
	while colinear:
		dest_index=random.randint(0,nCruise-1)
		wp_k=cruise_nodes[dest_index]
		pk=Point(coord_x[wp_k],coord_y[wp_k])
		colinear=False
		for i in range(nSkylanes):
			if list_skylanes[i].contains(p0) and list_skylanes[i].contains(pk): colinear=True
	# generate rest of the trajectory of intruder: calculate new nodes
	l = Line(p0,pk)
	waypoints=[]
	waypoints.append(wp_0)
	waypoints.append(wp_k)
	new_nodes=0 # number of new nodes from intersection of skylane and intruder's trajectory
	belongToSkylane={} # skylane to which each node belongs
	for node in cruise_nodes:
		for i in range(nSkylanes):
			if list_skylanes[i].contains(Point(coord_x[node],coord_y[node])):
				belongToSkylane[node]=i
				break
	onEdge={}# onEdge[node]=(u,v) if node belongs to edge (u,v)
	for i in range(nSkylanes):
		intersec=l.intersection(list_skylanes[i])
		#capire cosa da evalf
		p=intersec[0].evalf()
		closer=-1
		closer2=-1
		dist_closer=float('inf')
		dist_closer2=float('inf')
		for u in cruise_nodes:
			if belongToSkylane[u]==i:
				d=math.sqrt(pow(p[0]-coord_x[u],2)+pow(p[1]-coord_y[u],2))
				if d<dist_closer:
					dist_closer2=dist_closer
					closer2=closer
					dist_closer=d
					closer=u
				elif d<dist_closer2:
					dist_closer2=d
					closer2=u
				if(dist_closer<1 and dist_closer2<1): break # nodes at less than 1 minute travelling time found
		# only consider nodes between existing wps
		if dist_closer2+dist_closer< 1.0001 and dist_closer2+dist_closer>0.9999 and dist_closer>0.001: # dist_closer2+dist_closer~1 and candidate is not an existing node
			candidate_node=nNodes+new_nodes
			coord_x[candidate_node]=float(p[0])
			coord_y[candidate_node]=float(p[1])
			nodeType[candidate_node]=2
			isFATO[candidate_node]=False
			belongToSkylane[candidate_node]=i
			waypoints.append(candidate_node)
			if (closer,closer2) in edges: onEdge[candidate_node]=(closer,closer2)
			elif (closer2,closer) in edges: onEdge[candidate_node]=(closer2,closer)
			else: print("------------Scenario 3 generator: ERROR, closer nodes are not neighbours!!!")
			new_nodes+=1
		elif dist_closer<=0.001 and not closer in waypoints: waypoints.append(closer)
	if verbose:
		sys.stdout.write('Random nodes that define trajectory: %i and %i \n'%(wp_0,wp_k))
		print('New virtual nodes generated by intersection of skylane and intruder\'s trajectory')
		for i in range(new_nodes):
			sys.stdout.write('\t node %i in (%i,%i)\n'%(nNodes+i,onEdge[nNodes+i][0],onEdge[nNodes+i][1]))
	# insert new nodes into network, update distances, update paths of trips, update schedules of trips, update graph G
	for i in range(new_nodes):
		node=nNodes+i
		u=onEdge[node][0]
		v=onEdge[node][1]
		edges.remove(onEdge[node])
		edges.append((u,node))
		edges.append((node,v))
		dist.pop(onEdge[node])
		dist[(u,node)]=math.sqrt(pow(coord_x[node]-coord_x[u],2)+pow(coord_y[node]-coord_y[u],2))*v_default
		dist[(node,v)]=math.sqrt(pow(coord_x[node]-coord_x[v],2)+pow(coord_y[node]-coord_y[v],2))*v_default
		G.remove_edge(u,v)
		G.add_edge(u,node,dist=dist[(u,node)])
		G.add_edge(node,v,dist=dist[(node,v)])
		angle=crossing_angle(node,u,v)
		angleJunc[node,u,v]=angle
		angleJunc[node,v,u]=angle
		for uid in A:
			if (uid,u,v) in tuples_linkPath:
				tuples_linkPath.remove((uid,u,v))
				tuples_linkPath.append((uid,u,node))
				tuples_linkPath.append((uid,node,v))
				tuples_nodePath.append((uid,node))
				for w in range(nNodes):
					if (u,v,w) in angleJunc: # (u,w,v) in angleJunc as well
						angle=angleJunc[u,v,w]
						angleJunc.pop((u,v,w))
						angleJunc.pop((u,w,v))
						angleJunc[u,node,w]=angle
						angleJunc[u,w,node]=angle
					if (v,u,w) in angleJunc: # (v,w,u) in angleJunc as well
						angle=angleJunc[v,u,w]
						angleJunc.pop((v,u,w))
						angleJunc.pop((v,w,u))
						angleJunc[v,node,w]=angle
						angleJunc[v,w,node]=angle
				tplanMin[uid,node]=tplanMin[uid,u]+dist[(u,node)]/v_default
				tplanMax[uid,node]=tplanMax[uid,u]+dist[(u,node)]/v_default
				for trip in schedule['trips']:
					if trip['uid']==uid:
						numWP=len(trip['waypoints'])
						new_waypoints=(numWP+1)*[0]
						new_tEar=(numWP+1)*[0]
						new_tLat=(numWP+1)*[0]
						new_index=0
						for wp_index in range(numWP):
							new_waypoints[new_index]=trip['waypoints'][wp_index]
							new_tEar[new_index]=trip['tEarliest'][wp_index]
							new_tLat[new_index]=trip['tLatest'][wp_index]
							if trip['waypoints'][wp_index]==u:
								new_index+=1
								new_waypoints[new_index]=node
								new_tEar[new_index]=tplanMin[uid,node]
								new_tLat[new_index]=tplanMax[uid,node]
							new_index+=1
						trip['waypoints']=new_waypoints
						trip['tEarliest']=new_tEar
						trip['tLatest']=new_tLat
	nNodes=nNodes+new_nodes
	# order the list of waypoints
	# chose randomly if path top-bottom or bottom-top
	rand=random.uniform(0,1)
	initial_wp=0
	if rand < 0.5:
		min_y=float("inf")
		for wp in waypoints:
			if coord_y[wp]<min_y:
				min_y=coord_y[wp]
				initial_wp=wp
	else:
		max_y=0
		for wp in waypoints:
			if coord_y[wp]>max_y:
				max_y=coord_y[wp]
				initial_wp=wp
	# create path (ordered) and insert initial wp
	numWP=len(waypoints)
	path_intruder=[0]*numWP
	path_intruder[0]=initial_wp
	waypoints.remove(initial_wp)
	time_path_intruder=[0]*(numWP-1) #time_path_intruder[i]=travelling time(path_intruder[i],path_intruder[i+1])
	# insert the rest of wps
	for i in range(numWP-1):
		current_wp=path_intruder[i]
		next_wp=0
		dist_closer_wp=float("inf")
		for wp in waypoints:
			d=math.sqrt(pow(coord_x[wp]-coord_x[current_wp],2)+pow(coord_y[wp]-coord_y[current_wp],2))
			if d <dist_closer_wp:
				dist_closer_wp=d
				next_wp=wp
		path_intruder[i+1]=next_wp
		time_path_intruder[i]=dist_closer_wp # note: coordinates are travelling times!
		waypoints.remove(next_wp)
	#-------- Intruder's time schedule
	#timeTD=random.uniform(1, endHorizon/5) # previous version
	time_intru=math.ceil(random.uniform(11, endHorizon/2)) # 11 because max time ahead to react is 10
	timeTD=time_intru-nNonColIntru
	t_ear=[0]*numWP
	t_lat=[0]*numWP
	#t_ear[0]=math.ceil(timeTD) # previous version
	t_ear[0]=time_intru
	t_lat[0]=t_ear[0]+tErr
	for i in range(1,numWP):
		t_ear[i]=t_ear[i-1]+time_path_intruder[i-1]
		t_lat[i]=t_lat[i-1]+time_path_intruder[i-1]
	# --------Classify trips
	A.append(intru_uid)
	AP_NC.append(intru_uid)
	for trip in schedule['trips']:
		if trip['tEarliest'][0]>= timeTD and trip['uid']!=intru_uid:
			ground.append(trip['uid'])
		elif trip['tLatest'][len(trip['waypoints'])-1]<= timeTD:
			A.remove(trip['uid'])
		else:
			inflight.append(trip['uid'])
	#--------- Update variables and sets to include the new trajectory
	schedule['trips'].append({'uid':  intru_uid, "batch": -1,"waypoints" : path_intruder,"tEarliest" : t_ear,"tLatest" : t_lat})
	for wp_index in range(1,numWP-1): # angleJunc within path
		h=path_intruder[wp_index-1]
		m=path_intruder[wp_index]
		l=path_intruder[wp_index+1]
		angle=crossing_angle(m,h,l)
		angleJunc[m,h,l]=angle
		angleJunc[m,l,h]=angle
	for wp_index in range(numWP): # angleJunc between path and other nodes
		m=path_intruder[wp_index]
		for (u,v) in edges:
			h=-1
			if u==m and v in cruise_nodes: h=v
			elif v==m and u in cruise_nodes: h=u
			if h!=-1:
				if wp_index!=0:
					l=path_intruder[wp_index-1]
					angle=crossing_angle(m,h,l)
					angleJunc[m,h,l]=angle
					angleJunc[m,l,h]=angle
				if wp_index!=numWP-1:
					l=path_intruder[wp_index+1]
					angle=crossing_angle(m,h,l)
					angleJunc[m,h,l]=angle
					angleJunc[m,l,h]=angle
	for wp_index in range(numWP): # nodePath, tplan
		wp=path_intruder[wp_index]
		tuples_nodePath.append((intru_uid,wp))
		tplanMin[intru_uid,wp]=t_ear[wp_index]
		tplanMax[intru_uid,wp]=t_lat[wp_index]
	for i in range(len(path_intruder)-1): # edges, dist
		m=path_intruder[i]
		h=path_intruder[i+1]
		tuples_linkPath.append((intru_uid,m,h))
		if not (m,h) in edges:
			edges.append((m,h))
			dist[m,h]=time_path_intruder[i]
			G.add_edge(m,h,dist=dist[m,h])
	#if verbose:
	#	draw_skylane()
#------------ end function


A=[]
AP=[]
AP_NC=[]
inflight=[]
ground=[]
timeTD=-1
#--------------3a. Scenario 1: random disruptions
drifted=[]
delayed=[]
wpDrift=-1
timeDrifted=0
#--------------3b. Scenario 2: collaborative intruder
o_intru=-1
d_intru=-1

read_SD_schedule(input_file)

if nDrift+nDelay>0:
  disruptions_scenario1(nDrift,nDelay,int(math.floor(latestArrival_nominal)))
if nIntru>0:
  intruder_scenario2(int(math.floor(latestArrival_nominal)))
if nNonColIntru>0:
  intruder_scenario3(int(math.floor(latestArrival_nominal)))
file = open(output_file, "w")
# network details
nf = len(schedule["trips"])
#print nf and nn
file.write("param nn:= " + str(nNodes) +";\n" )
file.write("set F:=\n")
for trip in schedule["trips"]:
	file.write(str(trip["uid"]) + "\n")
#print arcs and distances
file.write(";\nparam: E: d:=\n")
for i in range(nEdges):
    file.write(
        "%i %i %f\n" % (edges[i][0], edges[i][1], dist[edges[i][0], edges[i][1]])
    )
#for i in range(nEdges):
#    file.write(
#        "%i %i %f\n" % (edges[i][1], edges[i][0], dist[edges[i][0], edges[i][1]])
#    )
#print starting points
file.write(";\nparam s:=\n" )
for i in range(nf):
    file.write(str(schedule["trips"][i]["uid"]) + " "+ str(schedule["trips"][i]["waypoints"][0])+ "\n")
#print ending points
file.write(";\nparam e:=\n" )
for i in range(nf):
    file.write(str(schedule["trips"][i]["uid"]) + " "+ str(schedule["trips"][i]["waypoints"][-1])+ "\n")
#print speed
file.write(";\nparam v_min:="+str(vmin) + "\n")
file.write(";\nparam v_max:=" +str(vmax) +" \n")

#print angles

file.write(";\nparam :conflictsNodes: angle:=\n")
for i, j, k in angleJunc:
	if angleJunc[i, j, k] >= math.pi/2:
		file.write("%i %i %i %f\n" % (i, j, k, 1))
	else:
		file.write("%i %i %i %f\n" % (i, j, k,1/math.sin( angleJunc[i, j, k])))
#print D
file.write(";\nparam D:=" + str(D) +";\n")
#print t_hat_ear
file.write("param t_hat_ear:=\n")
for trip in schedule["trips"]:
  for i in range(len(trip["tEarliest"])):
    if trip["uid"] in delayed and i == 0:
        file.write(str(trip["uid"]) + " " + str(trip["waypoints"][i]) + " " + str(timeTD) + "\n")
    elif trip["uid"] in drifted and trip["waypoints"][i] == wpDrift:
        file.write(str(trip["uid"]) + " " + str(trip["waypoints"][i]) + " " + str(trip["tEarliest"][i] + timeDrifted) + "\n")
    else :
        file.write(str(trip["uid"]) + " " + str(trip["waypoints"][i]) + " " + str(trip["tEarliest"][i]) + "\n")
file.write(";\nparam t_hat_lat:=\n")
for trip in schedule["trips"]:
  for i in range(len(trip["tLatest"])):
    if trip["uid"] in delayed and i == 0:
            file.write(str(trip["uid"]) + " " + str(trip["waypoints"][i]) + " " + str(timeTD + trip["tLatest"][i] - trip["tEarliest"][i]) + "\n")
    elif trip["uid"] in drifted and trip["waypoints"][i] == wpDrift:
        file.write(str(trip["uid"]) + " " + str(trip["waypoints"][i]) + " " + str(trip["tLatest"][i] + timeDrifted) + "\n")
    else :
        file.write(str(trip["uid"]) + " " + str(trip["waypoints"][i]) + " " + str(trip["tLatest"][i]) + "\n")
file.write(";\n")
#if nDrift > 0:
#    file.write("param drifted_flight:= " + str(drifted[0]) + ";\n")
#    file.write("param drifted_wp:= " + str(wpDrift) + ";\n")
#    for trip in schedule["trips"]:
#        if trip["uid"] in drifted:
#            uid = trip["uid"]
#            i = 0
#            for wp in trip["waypoints"]:
#                if wp == wpDrift:
#                    file.write("param drifted_t_ear_fix:= " + str(trip["tEarliest"][i] + timeDrifted) + ";\n")
#                    file.write("param drifted_t_lat_fix:= " + str(trip["tLatest"][i] + timeDrifted) + ";\n")
#                    break
#                i +=1
#            break
file.write("set fixedFlights :=\n")
for trip in schedule["trips"]:
  if (trip["uid"] in delayed) : 
    continue
  for i in range(len(trip["waypoints"])):
    if (i==0):
      continue
    file.write(str(trip["uid"]) + " " + str(trip["waypoints"][i-1]) + " "+ str(trip["waypoints"][i]) + "\n")

file.write(";\n")