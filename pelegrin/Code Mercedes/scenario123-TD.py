import math
import json
import random
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import sys
import re
from amplpy import AMPL
from sympy.geometry import Line
from sympy import Point
from sympy import Point2D

# input parameters
input_file=sys.argv[1]
output_file=sys.argv[2]
random_seed=int(sys.argv[3])
verbose=int(sys.argv[4]) # print outputs?
compareWithLocal=int(sys.argv[5]) # run AMPL only with "inConflict" pairs (not global approach)
nDrift=int(sys.argv[6]) # scenario 1: num drift
nDelay=int(sys.argv[7]) # scenario 1: num delayed
nIntru=int(sys.argv[8]) # scenario 2: if nIntru>0 then collaborative intruder. max time ahead before intruder's entrance < nIntru
nNonColIntru=int(sys.argv[9]) # scenario 3: if nNonColIntru>0 then non-collaborative intruder.  Idem
max_acceler=float(sys.argv[10]) # % max acceleration, affects q_max
max_deceler=float(sys.argv[11]) # % max deceleration, affects q_min
random.seed(random_seed)
# parameters from SD
D=0
v_default=0
v_climb=0
numBatches=0
tripsPerBatch=0
timeBatch=0
nNodes=0
nEdges=0
nFatos=0
coord_x={}
coord_y={}
nodeType={} # 0=take off junc (insertion); 1=landing junc (split); 2= inner junc (sync); 3=take off FATO; 4=landing FATO
isFATO={}
takeoffFATO_list=[]
landingFATO_list=[]
tuples_nodePath=[]
tuples_linkPath=[]
orig_list = {}
dest_list = {}
edges=[]
skylanes=[]
dist={}
angleJunc={}
tplanMin={}
tplanMax={}
schedule=json.loads("""{"trips":[]}""") # json list with trips
latestArrival_nominal=0
climb_slackSD=0
cruise_slackSD=0
# deconf parameters:
qmin_default=(100-max_deceler)/100
qmax_default=(100+max_acceler)/100
aheadMaxDef=2
delayMaxDef=2
vmin=0
vmax=0
# disruptions parameters:
epsilonDelay=0.1
epsilonDrift=0.5
min_trip_duration=7 # for intruder's path
max_trip_duration=25
# program parameters
G = nx.DiGraph()
tErr=1

###################################################################
#        1a. Function to read network from input file   		  #
###################################################################
def read_SD_schedule(fileName):
	global D
	global v_default
	global v_climb
	global numBatches
	global tripsPerBatch
	global timeBatch
	global nNodes
	global nEdges
	global nFatos
	global latestArrival_nominal
	global climb_slackSD
	global cruise_slackSD
	global vmin
	global vmax
	file=open(fileName)
	line=file.readline() # nNodes
	line_split=re.split('[:]', line)
	nNodes=int(line_split[1])
	line=file.readline() # nFatos
	line_split=re.split('[:]', line)
	nFatos=int(line_split[1])
	line=file.readline() # node list
	for i in range(nNodes):
		line=file.readline()
		entry=line.split()
		node=int(entry[0])
		code=int(entry[1])
		coord_x[node]=float(entry[2])
		coord_y[node]=float(entry[3])
		nodeType[node]=code
		if(code==3 or code==4): 
			isFATO[node]=1
			if code==3: takeoffFATO_list.append(node)
			else: landingFATO_list.append(node)
		else: isFATO[node]=0
	line=file.readline() # nEdges
	line_split=re.split('[:]', line)
	nEdges=int(line_split[1])
	line=file.readline() # edge list
	for i in range(nEdges):
		line=file.readline()
		entry=line.split()
		u=int(entry[0])
		v=int(entry[1])
		dist[u,v]=float(entry[2])
		edges.append((u,v))
		G.add_edge(u,v,dist=dist[u,v])
	line=file.readline() # nSkylanes
	line_split=re.split('[:]', line)
	nSkylanes=int(line_split[1])
	line=file.readline() # skylanes list
	for i in range(nSkylanes):
		line=file.readline()
		entry=line.split()
		skylanes.append((float(entry[0]),float(entry[1]),float(entry[2]),float(entry[3])))
	line=file.readline() # nAngles
	line_split=re.split('[:]', line)
	nAngles=int(line_split[1])
	line=file.readline() # angleJunction list
	for i in range(nAngles):
		line=file.readline()
		entry=line.split()
		angleJunc[int(entry[0]),int(entry[1]),int(entry[2])]=float(entry[3])
	line=file.readline() # D
	line_split=re.split('[:]', line)
	D=float(line_split[1])
	line=file.readline() # v_default
	line_split=re.split('[:]', line)
	v_default=float(line_split[1])
	line=file.readline() # v_climb
	line_split=re.split('[:]', line)
	v_climb=float(line_split[1])
	line=file.readline() # numBatches
	line_split=re.split('[:]', line)
	numBatches=int(line_split[1])
	line=file.readline() # tripsPerBatch
	line_split=re.split('[:]', line)
	tripsPerBatch=int(line_split[1])
	line=file.readline() # timeBatch
	line_split=re.split('[:]', line)
	timeBatch=float(line_split[1])
	line=file.readline() # nTrips
	line_split=re.split('[:]', line)
	nTrips=int(line_split[1])
	for i in range(nTrips):
		line=file.readline() # trip
		entry=line.split()
		uid=int(entry[1])
		line=file.readline() # batch
		line_split=re.split('[:]', line)
		batch=int(line_split[1])
		line=file.readline() # waypoints
		line_split=re.split('[:]', line)
		path=map(int,line_split[1].split(','))
		line=file.readline() # t_ear
		line_split=re.split('[:]', line)
		t_ear=map(float,line_split[1].split(','))
		line=file.readline() # t_lat
		line_split=re.split('[:]', line)
		t_lat=map(float,line_split[1].split(','))
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
#------------ end function

######################################################################
# 				 1b. Function to disrupt a batch   					#
#####################################################################
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
	for trip in schedule['trips']:
		A.append(trip['uid'])
	# create disruptions
	if nDelay>0: # there are delays
		if nDelay==1 and nDrift==0:
			delayed=random.sample(A,1)
			uid=delayed[0]
			timeTD=tplanMax[uid,orig_list[uid]]+epsilonDelay
		else:
			plannedByT={} # dict, plannedByT[t]=[3,6] if trips 3 and 6 have departure planned in [t-1,t]
			for t in range(1, endHorizon-3):
				for trip in schedule['trips']:
					if trip['tLatest'][0]==t:
						if t not in plannedByT:
							plannedByT[t]=[]
						plannedByT[t].append(trip['uid'])
			# check that nDelay<=  max{ len(plannedByT[t]) }
			maxLen_plannedByT=0
			for t in range(1, endHorizon-3):
				if t in plannedByT and len(plannedByT[t])> maxLen_plannedByT:
					maxLen_plannedByT=len(plannedByT[t])
			if maxLen_plannedByT< nDelay:
				print('WARNING: unfeasible number of delays (greater than simultaneous departures)')
				print('\t setting delays to %i'%maxLen_plannedByT)
				nDelay=maxLen_plannedByT
			# remove from plannedByT those lists with less trips than nDelay
			for t in range(1, endHorizon-3):
				if t in plannedByT and len(plannedByT[t])< nDelay:
					plannedByT.pop(t)
			# if there is also a drift, delay cannot happen at the beginning of time horizon
			if nDrift >0:
				for t in range(1, endHorizon/5):
					if t in plannedByT:
						plannedByT.pop(t)
			tDelay=random.choice([t for t in plannedByT])
			delayed=random.sample(plannedByT[tDelay],nDelay)
			timeTD=tDelay+epsilonDelay
	if nDrift >0: # precondition: if this happens we have 1 drift (and 0 or 1 delay)
		drifted=random.sample(set(A)-set(delayed),nDrift)
		if nDelay >0: # make sure drift and delay match
			notFound=True
			while(notFound):
				uid=drifted[0]
				if tplanMin[uid,orig_list[uid]]>= timeTD or tplanMin[uid,dest_list[uid]]<= timeTD:
					# this trip is on ground, cannot be drifted. Chose another one:
					drifted=random.sample(set(A)-set(delayed),1)
				else:
					for trip_drift in schedule['trips']:
						if trip_drift['uid']==uid:
							for wp_index in range(len(trip_drift['waypoints'])):
								if trip_drift['tEarliest'][wp_index]>= timeTD:
									notFound=False
									wpDrift=trip_drift['waypoints'][wp_index]
									break
							break
		else: # nDrift=1 and nDelay=0
			for trip_drift in schedule['trips']:
				if trip_drift['uid']==drifted[0]:
					wpDrift_index=random.randint(2,len(trip_drift['waypoints'])-2)
					wpDrift=trip_drift['waypoints'][wpDrift_index]
					timeTD=trip_drift['tEarliest'][wpDrift_index]-epsilonDrift
					break
		uid=drifted[0]
		rand=random.uniform(0,1) # ahead or delay?
		if rand < 0.5: tminNew=random.uniform(timeTD+epsilonDrift/2, tplanMin[uid,wpDrift]-0.01)  # ahead
		else:  tminNew=random.uniform(tplanMin[uid,wpDrift]+0.01, tplanMin[uid,wpDrift]+epsilonDrift/2) # delay
		tminNew=math.ceil(tminNew*100)/100
		timeDrifted=tminNew-tplanMin[uid,wpDrift]# if negative ahead, otherwise delay
		print(tminNew,tplanMin[uid,wpDrift],timeDrifted)
	# classify trips
	for trip in schedule['trips']:
		if trip['tEarliest'][0]>= timeTD or trip['uid'] in delayed:
			ground.append(trip['uid'])
		elif trip['tLatest'][len(trip['waypoints'])-1]<= timeTD: 
			A.remove(trip['uid'])
		else:
			inflight.append(trip['uid'])
#------------ end function

######################################################################
# 			 1c. Function to create collab. intruder				#
#####################################################################
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
	nSkyports=nFatos/2;
	orig_index=random.randint(0,nSkyports-1)
	o_intru=takeoffFATO_list[orig_index]
	d_intru=o_intru
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

	# # Generate all possible intruder's path by brute force:
	# # for origin in range(0,nSkyports):
	# # 	for destination in range(0,nSkyports):
	# # 		if origin != destination:
	# # 			o=takeoffFATO_list[origin]
	# # 			d=landingFATO_list[origin]
	# # 			path=nx.shortest_path(G,o, d, weight='dist') # network weight is the distance
	# # 			path_distance=nx.shortest_path_length(G,o, d, weight='dist')
	# # 			trip_duration=path_distance/v_default
	# # 			if trip_duration >= min_trip_duration and trip_duration <= max_trip_duration:
	# # 				numWP=len(path)
	# # 				for time in range(0,int(round(numBatches*timeBatch))):
	# # 					intruder_schedule=copy.deepcopy(schedule)
	# # 					t_earliest=[0]*numWP
	# # 					t_latest=[0]*numWP
	# # 					t_earliest[0]=time
	# # 					t_latest[0]=time+tErr
	# # 					t_earliest[1]=t_earliest[0]+G[path[0]][path[1]]['dist']/v_climb
	# # 					t_latest[1]=t_latest[0]+G[path[0]][path[1]]['dist']/v_climb
	# # 					for i in range(2,numWP-1):
	# # 						t_earliest[i]=t_earliest[i-1]+G[path[i-1]][path[i]]['dist']/v_default
	# # 						t_latest[i]=t_latest[i-1]+G[path[i-1]][path[i]]['dist']/v_default
	# # 					t_earliest[numWP-1]=t_earliest[numWP-2]+G[path[numWP-2]][path[numWP-1]]['dist']/v_climb
	# # 					t_latest[numWP-1]=t_latest[numWP-2]+G[path[numWP-2]][path[numWP-1]]['dist']/v_climb	
	# # 					uid= nTrips-1
	# # 					trip_duration=t_latest[numWP-1]-t_earliest[0]
	# # 					intruder_schedule['trips'].append({'uid':  uid, "waypoints" : path, "timeEarly": t_earliest,"timeLate": t_latest, "duration": trip_duration, "intruder": 'Y'})
	# # 					create_ampldat_file(intruder_schedule,dataFileName+"-intru-"+str(o)+"-"+str(d)+"-"+str(time)+".dat")
#------------ end function

######################################################################
# 			 1d. Function to create non-collab. intruder			#
#####################################################################
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

######################################################################
#  				1e. Function to set AMPL TD model data          	#
#####################################################################		
def load_AMPL_data(ampl):
	# fix data (remove elements not in A)
	to_remove=[]
 	for (i,m,h) in tuples_linkPath:
 		if i not in A:
 			to_remove.append((i,m,h))
 	for (i,m,h) in to_remove:
 		tuples_linkPath.remove((i,m,h))
 	to_remove=[]
 	for (i,m) in tuples_nodePath:
 		if i not in A:
 			to_remove.append((i,m))
 	for (i,m) in to_remove:
 		tuples_nodePath.remove((i,m))
 		tplanMin.pop((i,m))
 		tplanMax.pop((i,m))
 	to_remove=[]
 	for i in orig_list:
 		if i not in A:
 			to_remove.append(i)
 	for i in to_remove:
 		orig_list.pop(i)
 		dest_list.pop(i)
	# SETS
	A_ampl = ampl.getSet('A')
 	A_ampl.setValues(A)
 	AP_ampl = ampl.getSet('AP')
 	AP_ampl.setValues(AP)
 	AP_NC_ampl = ampl.getSet('AP_NC')
 	AP_NC_ampl.setValues(AP_NC)
 	GROUND_ampl = ampl.getSet('GROUND')
 	GROUND_ampl.setValues(ground)
 	DELAYED_ampl = ampl.getSet('DELAYED')
 	DELAYED_ampl.setValues(delayed)
 	NODES = ampl.getSet('NODES')
 	NODES.setValues(range(nNodes))
 	FATOS = ampl.getSet('FATOS')
 	FATOS.setValues(takeoffFATO_list+landingFATO_list)
 	LINKS = ampl.getSet('LINKS')
 	LINKS.setValues(edges)
 	LINKSPATH = ampl.getSet('LINKSPATH')
 	LINKSPATH.setValues(tuples_linkPath)
 	NODESPATH = ampl.getSet('NODESPATH')
 	NODESPATH.setValues(tuples_nodePath)
 	# PARAMETERS
 	tplanMin_ampl=ampl.getParameter('tplanMin')
 	tplanMax_ampl=ampl.getParameter('tplanMax')
 	tplanMin_ampl.setValues(tplanMin)
 	tplanMax_ampl.setValues(tplanMax)
 	orig_ampl=ampl.getParameter('orig')
 	dest_ampl=ampl.getParameter('dest')
 	orig_ampl.setValues(orig_list)
 	dest_ampl.setValues(dest_list)
 	D_ampl=ampl.getParameter('D')
 	tini_ampl=ampl.getParameter('tini')
 	vclimb_ampl=ampl.getParameter('vclimb')
 	vmin_ampl=ampl.getParameter('vmin')
 	vmax_ampl=ampl.getParameter('vmax')
 	D_ampl.set(D)
 	tini_ampl.set(timeTD)
 	vclimb_ampl.set(v_climb)
 	vmin_ampl.set(vmin)
 	vmax_ampl.set(vmax)
 	dist_ampl=ampl.getParameter('dist')
 	angleJunc_ampl=ampl.getParameter('angleJunction')
 	dist_ampl.setValues(dist)
 	angleJunc_ampl.setValues(angleJunc)
#------------ end function

######################################################################
#  			1f. Functions to modify AMPL model     		  		   	#
#####################################################################		
def include_drift(ampl):
	global timeDrifted

	var_t_earliest= ampl.getVariable('tmin')
	var_t_latest= ampl.getVariable('tmax')
	uid=drifted[0]
	tminOld=tplanMin[uid,wpDrift]
	tminNew=tplanMin[uid,wpDrift]+timeDrifted
	var_t_earliest[uid,wpDrift].fix(tminNew)
	var_t_latest[uid,wpDrift].fix(tminNew+tplanMax[uid,wpDrift]-tplanMin[uid,wpDrift])
	const1=ampl.getConstraint('tmin_past')
	const2=ampl.getConstraint('tmax_past')
	const1[uid,wpDrift].drop()
	const2[uid,wpDrift].drop()
	if verbose:
		print('time of the drift',timeDrifted)
#------------ end function

def set_AMPL_model(ampl,modifiable_trips):
	var_dev= ampl.getVariable('dev')
	WP_DEV_ampl = ampl.getSet('WP_DEV')
	const_lb=ampl.getConstraint('LBt')
	const_ub=ampl.getConstraint('UBt')
	NOTFIXED_NODESPATH_ampl = ampl.getSet('NOTFIXED_NODESPATH')
	var_t_earliest= ampl.getVariable('tmin')
	var_t_latest= ampl.getVariable('tmax')
	#const=ampl.getConstraint('fixDev')
	if verbose:
	 	print('Non-intruders modifiable_trips: ',modifiable_trips)
	#for i in modifiable_trips: # fix all trips that are not directly in conflict
	# 	for (uid,wp) in WP_DEV_ampl.members():
	# 		if uid==i: 
	# 			const[i,wp].drop()
	for (uid,wp) in WP_DEV_ampl.members():
		if not uid in modifiable_trips:	
			var_dev[uid,wp].fix(0)
	# for (uid,wp) in NOTFIXED_NODESPATH_ampl.members():
	#  	if not uid in modifiable_trips and wp==dest_list[uid]:	
	#  		const_lb[uid,wp].drop()
	#  		const_ub[uid,wp].drop()
	# 		var_t_earliest[uid,wp].fix(tplanMin[uid,wp])
	# 		var_t_latest[uid,wp].fix(tplanMax[uid,wp])
	ampl.option['solver'] = 'cplex'
	#ampl.setOption('cplex_options',"mipgap=0.000000001") 
	#ampl.setOption('cplex_options',"absmipgap=0") 
	#ampl.setOption('cplex_options',"feasibility =0.000000001") 
	#ampl.setOption('cplex_options',"integrality=0.000000001") 
#------------ end function

######################################################################
#  	1g. Functions to process solution and generate output        	#
#####################################################################	
# ---------- Auxiliary func: count number of conflicts	
def count_conflicts(ampl,num_conflicts,inConflict_trips,conflict_generators):
	local_tplanMin=tplanMin.copy()
	local_tplanMax=tplanMax.copy()
	# identify those trips that are source of potential conflicts
	if nIntru>0:
		conflict_generators.append(AP[0])
	if nNonColIntru>0:
		conflict_generators.append(AP_NC[0])
	# fix a tentative schedule for those trips (an ideal one)
	if nDelay>0:
		for uid_delayed in delayed:
			conflict_generators.append(uid_delayed)
			#print('*** tentative schedule for %i'%uid_delayed)
			for trip in schedule['trips']:
				uid=trip['uid']
				if uid==uid_delayed:
					for wp_index in range(len(trip['waypoints'])):
						wp=trip['waypoints'][wp_index]
						if wp_index==0:
							local_tplanMin[uid,wp]+=2
							local_tplanMax[uid,wp]+=2
						elif wp_index==1: # climbing does not allow speed regulation
							local_tplanMin[uid,wp]+=2
							local_tplanMax[uid,wp]+=2
						else:
							wp_previous=trip['waypoints'][wp_index-1]
							time_traverse_edge= dist[wp_previous,wp]/v_default
							min_time_traverse_edge=time_traverse_edge/qmax_default
							width=local_tplanMax[uid,wp]-local_tplanMin[uid,wp]
							local_tplanMin[uid,wp]=max(local_tplanMin[uid,wp],local_tplanMin[uid,wp_previous]+min_time_traverse_edge)
							local_tplanMax[uid,wp]=local_tplanMin[uid,wp]+width
						#sys.stdout.write('\t wp %i: [%.5f,%.5f]\n'%(wp,local_tplanMin[uid,wp],local_tplanMax[uid,wp]))
					break 
	if nDrift>0:
		for uid_drifted in drifted:	
			conflict_generators.append(uid_drifted)
			for trip in schedule['trips']:
				uid=trip['uid']
				if uid==uid_drifted:
					isDrifted=False
					for wp_index in range(len(trip['waypoints'])):
						wp=trip['waypoints'][wp_index]
						if wpDrift==wp:
							isDrifted=True
							local_tplanMin[uid,wp]=local_tplanMin[uid,wp]+timeDrifted
							local_tplanMax[uid,wp]=local_tplanMax[uid,wp]+timeDrifted
						elif isDrifted:	
							wp_previous=trip['waypoints'][wp_index-1]
							time_traverse_edge= dist[wp_previous,wp]/v_default
							min_time_traverse_edge=time_traverse_edge/qmax_default
							max_time_traverse_edge=time_traverse_edge/qmin_default
							width=local_tplanMax[uid,wp]-local_tplanMin[uid,wp]
							if timeDrifted>0:# if positive, delay
								local_tplanMin[uid,wp]=max(local_tplanMin[uid,wp],local_tplanMin[uid,wp_previous]+min_time_traverse_edge)
							else: # ahead
								local_tplanMin[uid,wp]=min(local_tplanMin[uid,wp],local_tplanMin[uid,wp_previous]+max_time_traverse_edge)
							local_tplanMax[uid,wp]=local_tplanMin[uid,wp]+width
					break
	# store S_angle for calculations
	S_angle={}
	for (m,h,l) in angleJunc:
		if angleJunc[m,h,l]>= 1.570796: S_angle[m,h,l]=1 
		else: S_angle[m,h,l]=1/math.sin(angleJunc[m,h,l])
	# count conflicts of each type
	print('##    Conflicts generated:')
	for (i,j,m,h) in ampl.getSet('TRAIL_LAND').getValues():
		if (i in conflict_generators or j in conflict_generators) and local_tplanMin[j,m]-local_tplanMax[i,m] < 1:
			num_conflicts[0]+=1
			inConflict_trips.append(i)
			inConflict_trips.append(j)
			if verbose: 
				sys.stdout.write('\ttrail_land: (%i,%i,%i,%i) tplanMin[%i,%i]-tplanMax[%i,%i]=%f-%f< 1\n'
										%(i,j,m,h,j,m,i,m,local_tplanMin[j,m],local_tplanMax[i,m]))
	for (i,j,m,h) in ampl.getSet('TRAIL_TAKEOFF').getValues():
		if  (i in conflict_generators or j in conflict_generators) and local_tplanMin[j,m]-local_tplanMax[i,m] < 1: 
			num_conflicts[1]+=1
			inConflict_trips.append(i)
			inConflict_trips.append(j)
			if verbose:
				sys.stdout.write('\ttrail_takeoff: (%i,%i,%i,%i) tplanMin[%i,%i]-tplanMax[%i,%i]=%f-%f< 1 \n'
					%(i,j,m,h,j,m,i,m,local_tplanMin[j,m],local_tplanMax[i,m]))
	for (i,j,m,h) in ampl.getSet('TRAIL_CRUISE').getValues():
		if  (i in conflict_generators or j in conflict_generators) and (local_tplanMin[j,m]-local_tplanMax[i,m] < D/vmin or local_tplanMin[j,h]-local_tplanMax[i,h] < D/vmin): 
			num_conflicts[2]+=1
			inConflict_trips.append(i)
			inConflict_trips.append(j)
			if verbose:
				sys.stdout.write('\ttrail_cruise: (%i,%i,%i,%i)\n'%(i,j,m,h))
	for (i,j,m,h_i,l_i,h_j,l_j) in ampl.getSet('CROSS_CONF').getValues():
		if  (i in conflict_generators or j in conflict_generators) and (local_tplanMin[j,m]-local_tplanMax[i,m]<(D/vmin)*S_angle[m,h_i,h_j] or
								local_tplanMin[j,m]-local_tplanMax[i,m]<(2*D/vmin) * S_angle[m,h_j,l_i] or
								local_tplanMin[j,m]-local_tplanMax[i,m]<(D/vmin) * S_angle[m,l_i,l_j]):
			num_conflicts[3]+=1
			inConflict_trips.append(i)
			inConflict_trips.append(j)
			if verbose:
				sys.stdout.write('\tcross_conf: (%i,%i,%i,%i,%i,%i,%i)\n'%(i,j,m,h_i,l_i,h_j,l_j))
	for (i,j,m,h_i,h_j,l) in ampl.getSet('MERGE_CONF').getValues():
		if  (i in conflict_generators or j in conflict_generators) and local_tplanMin[j,m]-local_tplanMax[i,m]<(D/vmin)*S_angle[m,h_i,h_j]:
			num_conflicts[4]+=1
			inConflict_trips.append(i)
			inConflict_trips.append(j)
			if verbose:
				sys.stdout.write('\tmerge_conf: (%i,%i,%i,%i,%i,%i)\n'%(i,j,m,h_i,h_j,l))
	for (i,j,m,h,l_i,l_j) in ampl.getSet('SPLIT_CONF').getValues():
		if  (i in conflict_generators or j in conflict_generators) and local_tplanMin[j,m]-local_tplanMax[i,m]<(D/vmin)*S_angle[m,l_i,l_j]:
			num_conflicts[5]+=1
			inConflict_trips.append(i)
			inConflict_trips.append(j)
			if verbose:
				sys.stdout.write('\tsplit_conf: (%i,%i,%i,%i,%i,%i)\n'%(i,j,m,h,l_i,l_j))
	for (i,j,m,h_i,h_j) in ampl.getSet('MERGE_LAND').getValues():
		if  (i in conflict_generators or j in conflict_generators) and local_tplanMin[j,m]-local_tplanMax[i,m] < 1: 
			num_conflicts[6]+=1
			inConflict_trips.append(i)
			inConflict_trips.append(j)
			if verbose:
				sys.stdout.write('\tmerge_land: (%i,%i,%i,%i,%i) tplanMin[%i,%i]-tplanMax[%i,%i]=%f-%f< 1\n'
					%(i,j,m,h_i,h_j,j,m,i,m,local_tplanMin[j,m],local_tplanMax[i,m]))
	for (i,j,m,l_i,l_j) in ampl.getSet('SPLIT_TAKEOFF').getValues():
		if  (i in conflict_generators or j in conflict_generators) and local_tplanMin[j,m]-local_tplanMax[i,m] < 1: 
			num_conflicts[7]+=1
			inConflict_trips.append(i)
			inConflict_trips.append(j)
			if verbose:
				sys.stdout.write('\tsplit_takeoff: (%i,%i,%i,%i,%i) tplanMin[%i,%i]-tplanMax[%i,%i]=%f-%f< 1\n'
					%(i,j,m,l_i,l_j,j,m,i,m,local_tplanMin[j,m],local_tplanMax[i,m]))		
#------------ end function

# ---------- Auxiliary func: generate R plot to file
def R_plot(list_trips,key1,key2,fileName):
	original=sys.stdout 
	sys.stdout=open(fileName,'a')
	sys.stdout.write('library(colorspace)\n')
	list_nodes=[]
	maxT=0
	# store traversed wp
	for uid in list_trips:
		for trip in schedule['trips']:
			if trip['uid']==uid:
				for wp in trip['waypoints']:
					if not wp in list_nodes:
						list_nodes.append(wp)
				break
	nNodes_traversed=len(list_nodes)

	for uid in list_trips:
		for trip in schedule['trips']:
			if trip['uid']==uid:
				sys.stdout.write('nodes%i=c('%uid)
				sys.stdout.write('%i'%list_nodes.index(trip['waypoints'][0]))
				for wp_index in range(1,len(trip['waypoints'])):
					wp=trip['waypoints'][wp_index]
					sys.stdout.write(',%i'%list_nodes.index(wp))
				sys.stdout.write(')\n')
				sys.stdout.write('tear%i=c('%uid)
				sys.stdout.write('%f'%trip[key1][0])
				for wp_index in range(1,len(trip['waypoints'])):
					sys.stdout.write(',%f'%trip[key1][wp_index])
				sys.stdout.write(')\n')
				sys.stdout.write('tlat%i=c('%uid)
				sys.stdout.write('%f'%trip[key2][0])
				for wp_index in range(1,len(trip['waypoints'])):
					time=trip[key2][wp_index]
					sys.stdout.write(',%f'%time)
					if time > maxT:
						maxT=time
				sys.stdout.write(')\n')
				break

	sys.stdout.write('nNodes=%i\n'%nNodes_traversed)
	sys.stdout.write('maxT=%i\n'%maxT)

	sys.stdout.write('tear_trips=list(tear%i'%list_trips[0])
	for uid in list_trips:
		if uid!=list_trips[0]:
			sys.stdout.write(',tear%i'%uid)
	sys.stdout.write(')\n')
	sys.stdout.write('tlat_trips=list(tlat%i'%list_trips[0])
	for uid in list_trips:
		if uid!=list_trips[0]:
			sys.stdout.write(',tlat%i'%uid)
	sys.stdout.write(')\n')
	sys.stdout.write('nodes_trips=list(nodes%i'%list_trips[0])
	for uid in list_trips:
		if uid!=list_trips[0]:
			sys.stdout.write(',nodes%i'%uid)
	sys.stdout.write(')\n')
	sys.stdout.write('nTrips=length(nodes_trips)\n')
	sys.stdout.write('colors_vector=rainbow_hcl(nTrips)\n')
	sys.stdout.write('plot(1, type="n",xlab="time",ylab="node",xlim=c(0,maxT+1),ylim=c(0,nNodes),lwd=3)\n')
	sys.stdout.write('for (i in 1:nTrips){\n')
	sys.stdout.write('\ttear=as.numeric(unlist(tear_trips[i]))\n')
	sys.stdout.write('\ttlat=as.numeric(unlist(tlat_trips[i]))\n')
	sys.stdout.write('\tnodes=as.numeric(unlist(nodes_trips[i]))\n')
	sys.stdout.write('\tlines(tear,nodes,type="p",col=colors_vector[i], xlab="time",ylab="node",xlim=c(0,maxT+1),ylim=c(0,nNodes),lwd=3)\n')
	sys.stdout.write('\tlines(tlat,nodes,type="p",col=colors_vector[i], xlab="time",ylab="node",xlim=c(0,maxT+1),ylim=c(0,nNodes),lwd=3)\n')
	sys.stdout.write('\tlines(tear,nodes,type="l",lty="dashed",col=colors_vector[i],lwd=3)\n')
	sys.stdout.write('\tlines(tlat,nodes,type="l",lty="dashed",col=colors_vector[i],lwd=3)\n')
	sys.stdout.write('\tfor(k in 1:length(nodes)){\n')
	sys.stdout.write('\t\tsegments(tear[k], nodes[k],tlat[k], nodes[k], col=colors_vector[i],lwd=3)\n')
	sys.stdout.write('\t}\n')
	sys.stdout.write('}\n')
	sys.stdout.write('for (i in 1:nNodes){\n')
	sys.stdout.write('abline(h = i-1,lty=2)\n')
	sys.stdout.write('}\n')
	sys.stdout.write('legend_vector=rep("",nTrips)\n')
	for uid_index in range(len(list_trips)):
		sys.stdout.write('legend_vector[%i]=paste("trip",%i)\n'%(uid_index+1,list_trips[uid_index]))
	sys.stdout.write('legend(maxT-1, nNodes/2, legend=legend_vector, lty=1,lwd=2,col=colors_vector, cex = 0.6)\n')
	sys.stdout.write('###                   END\n')
	sys.stdout=original
#------------ end function

# ---------- Auxiliary func: print summary to console
def print_solution(ampl,code,isLocal):
	var_t_lb= ampl.getVariable('t_lb')
	var_t_ub= ampl.getVariable('t_ub')
	var_t_earliest= ampl.getVariable('tmin')
	var_t_latest= ampl.getVariable('tmax')
	var_dev= ampl.getVariable('dev')
	WP_DEV_ampl = ampl.getSet('WP_DEV')
	print('\t uid\twp\tplan\tnew \ttmin in [lb,ub]\tdev')
	for trip in schedule['trips']:
		uid=int(trip['uid'])
		numWP=len(trip['waypoints'])
		for wp_index in range(numWP):
			wp=int(trip['waypoints'][wp_index])
			if (uid, wp) in WP_DEV_ampl.members():	
				dev=var_dev[uid,wp].value()
				condition=False
				if (code >=0) and wp_index==code: condition=True
				elif (code ==-100) and wp_index!=0 and wp_index!=1 and wp_index!=numWP-2 and wp_index!=numWP-1: condition=True
				elif (code ==-1 or code==-2) and wp_index==numWP+code: condition=True
				if dev>=0.00001 and condition:
					sys.stdout.write('\t%i\t%i\t[%.2f,%.2f]'%(uid, wp, 
							trip['tEarliest'][wp_index], trip['tLatest'][wp_index]))
					if isLocal: sys.stdout.write('\t[%.2f,%.2f]'%(trip['local_tEarliest'][wp_index], trip['local_tLatest'][wp_index]))	
					else:sys.stdout.write('\t[%.2f,%.2f]'%(trip['new_tEarliest'][wp_index], trip['new_tLatest'][wp_index]))	
					sys.stdout.write('\t[%.3f,%.3f]\t[%.3f,%.3f]\t%.3f\n'%( 
							var_t_earliest[uid,wp].value(),var_t_latest[uid,wp].value(),
							var_t_lb[uid,wp].value(),var_t_ub[uid,wp].value(),dev))	
#------------ end function

# ---------- Main func: output info about the instance
def instance_info_output(ampl):
	global inConflict_trips
	global conflict_generators
	global input_file
	global climb_slackSD
	global cruise_slackSD
	global random_seed
	global nNodes
	global nFatos
	global nDrift
	global nDelay
	global timeDrifted
	global timeTD
	global o_intru
	global d_intru
	# potential number of conflicts
	num_conflicts={0:0,1:0,2:0,3:0,4:0,5:0,6:0,7:0}
	count_conflicts(ampl,num_conflicts,inConflict_trips,conflict_generators) ## WARNING!! This method changes the values of tplanMin and tplanMax
	inConflict_trips=set(inConflict_trips)-set(conflict_generators)
	inConflict_trips=set(map(int,inConflict_trips))
	if verbose:
		print('inConflict_trips:',inConflict_trips)
	trail_land=num_conflicts[0]
	trail_takeoff=num_conflicts[1]
	trail=num_conflicts[2]
	cross=num_conflicts[3]
	merge=num_conflicts[4]
	split=num_conflicts[5]
	merge_land=num_conflicts[6]
	split_takeoff=num_conflicts[7]
	# Summary to file
	original=sys.stdout 
	sys.stdout=open(output_file,'a')
	sys.stdout.write('\n%s\t%.5f\t%.5f\t%i\t%i\t%i'%(input_file,climb_slackSD,cruise_slackSD,random_seed,nNodes,nFatos))
	sys.stdout.write('\t%i\t%i\t%i\t%i\t%i\t%i'%(len(A),len(inflight),nDelay,nDrift,nIntru,nNonColIntru))
	sys.stdout.write('\t%i\t%i\t%.2f\t%.5f\t%.2f'%(o_intru,d_intru,timeTD,timeDrifted,latestArrival_nominal))
	sys.stdout.write('\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i'%(trail_land, trail_takeoff, trail, cross, merge, 
															split, merge_land, split_takeoff,len(inConflict_trips)))
	sys.stdout=original
#------------ end function

# Main func:  output info about solution (and process this solution: count touched flights, rounding, etc)
def solution_postprocess_and_output(ampl,isLocal):
	global schedule
	global latestArrival_nominal
	# Objective value and status
	obj=ampl.getObjective('Dev') 
	objVal=obj.value()
	status=obj.result()
	# Priority traffic: store delay 
	var_delayPriority= ampl.getVariable('delayPriority')
	totalDelay_Priority=0
	for i in AP:
		totalDelay_Priority+=var_delayPriority[i].value()
	# Non-priority traffic: store deviations and delays. Store rounded solution
	var_dev= ampl.getVariable('dev')
	var_t_earliest= ampl.getVariable('tmin')
	var_t_latest= ampl.getVariable('tmax')
	NOTFIXED_NODESPATH_ampl = ampl.getSet('NOTFIXED_NODESPATH')
	NODESPATH_ampl = ampl.getSet('NODESPATH')
	WP_DEV_ampl = ampl.getSet('WP_DEV')
	totalDev_nonPriority=0 # total deviation
	totalDelay_destination=0 # delay at destination of non-priority traffic
	totalDelay_departure=0 # sum of delays at departure of non priority traffic
	num_delayedDeparture=0 # num trips that were delayed (all of them, even if delay is part of disruption)
	num_delayedDestination=0 # num trips that arrive with delay (all of them, even if delay is part of disruption)
	num_touchedTrips=0 # num trips such that the solution does not coincide with nominal plan 
	num_touchedRounded=0 # num trips such that the rounded solution does not coincide with nominal plan 
	list_touched=[] # list of touched trips
	list_touchedRounded=[] # list of touched trips if consider rounded schedules
	rounded_totalDev_nonPriority=0
	rounded_totalDelay_destination=0
	latestArrival_new=0
	max_dev=0 # maximum deviation from plan at waypoint of non priority traffic
	max_delay_o=0 # maximum delay at origin
	max_delay_d=0 # maximum delay at destination
	wp_max_delay_o=-1 # wp of maximum delay at origin
	wp_max_delay_d=-1 # wp of maximum delay at destination
	rounded_max_dev=0 # maximum deviation from plan at waypoint after rounding
	wp_max_dev=-1 # waypoint at which max dev is achieved
	wp_rounded_max_dev=-1
	dev_wp=nNodes*[0] # dev_wp[i]= total deviation of trips at wp
	for trip in schedule['trips']:
		uid=int(trip['uid'])
		if uid in A and not uid in AP_NC:	
			# Initialization of local variables for this trip
			numWP=len(trip['waypoints'])
			new_t_ear=[0]*numWP
			new_t_lat=[0]*numWP
			touched_rounded_this=False	
			touched_this=False
			# Departure
			wp=int(trip['waypoints'][0])
			departure_ear=round(var_t_earliest[uid,wp].value())
			departure_lat=round(var_t_latest[uid,wp].value())
			new_t_ear[0]=departure_ear
			new_t_lat[0]=departure_lat
			delay=departure_ear-tplanMin[uid,wp]
			if delay>=0.000000001:
				if not uid in AP:
					totalDev_nonPriority+=delay
				num_delayedDeparture+=1
				num_touchedTrips+=1
				list_touched.append(uid)
				totalDelay_departure+=delay
				touched_this=True
				if delay> max_dev:
					max_dev=delay
					wp_max_dev=wp
				if delay> max_delay_o:
					max_delay_o=delay
					wp_max_delay_o=wp
			# Waypoints
			for wp_index in range(1,numWP):
				wp=int(trip['waypoints'][wp_index])
				# we need to check (uid,wp) is in some trip (there could be deleted trips, those that were finished)
				if (uid, wp) in NODESPATH_ampl.members():	
					# store rounded solution
					new_t_ear[wp_index]=math.ceil(var_t_earliest[uid,wp].value()*100)/100
					new_t_lat[wp_index]=math.floor(var_t_latest[uid,wp].value()*100)/100
					# count rounded deviations
					if touched_rounded_this==False and (new_t_ear[wp_index]!=tplanMin[uid,wp] or new_t_lat[wp_index]!=tplanMax[uid,wp]):
						touched_rounded_this=True
						num_touchedRounded+=1
						list_touchedRounded.append(uid)
						if verbose:
							print('rounded touched',uid,wp,tplanMin[uid,wp],new_t_ear[wp_index],tplanMax[uid,wp],new_t_lat[wp_index])
					dev_round=abs(new_t_ear[wp_index]-tplanMin[uid,wp])
					rounded_totalDev_nonPriority+=dev_round
					if dev_round> rounded_max_dev:
						rounded_max_dev=dev_round
						wp_rounded_max_dev=wp
					if wp_index==numWP-1:
						rounded_totalDelay_destination+=dev_round 
						if new_t_lat[wp_index]> latestArrival_new:
							latestArrival_new=new_t_lat[wp_index]
					# count number of touched trips and deviations
					if (uid,wp) in WP_DEV_ampl.members():
						dev=var_dev[uid,wp].value()
						if dev>=0.00001:
							dev_wp[wp]+=dev
							if dev> max_dev:
								max_dev=dev
								wp_max_dev=wp
							totalDev_nonPriority+=dev
							if touched_this==False: # the trip is not delayed and this is the first wp with dev
								list_touched.append(uid)
								touched_this=True
								num_touchedTrips+=1	
							if wp_index==numWP-2:
								totalDelay_destination+=dev	
								num_delayedDestination+=1
								if dev> max_delay_d:
									max_delay_d=dev
									wp_max_delay_d=wp
			if isLocal==False:
				trip['new_tEarliest']=new_t_ear
				trip['new_tLatest']=new_t_lat
			else:
				trip['local_tEarliest']=new_t_ear
				trip['local_tLatest']=new_t_lat
	max_totalDev_at_wp=max(dev_wp)
	wp_maxTotalDev=np.argmax(dev_wp)
	# Non-priority traffic: store speed adjustments
	total_speedAdjustments=0 # number of seed adjustments in the solution
	num_tripsSpeedAdjusted=0 # num trips that received only speed adjustments
	speedAdjustment_driftedTrip=0 # number of speed adjustments aplied to the drifted trip (if any)
	accelerations=int(math.ceil(max_acceler))*[0] # accelerations[i]= number of times an acceler in (i, i+1] % is applied
	decelerations=int(math.ceil(max_deceler))*[0] # decelerations[i]= number of times a deceler in (i, i+1] %  is applied
	for trip in schedule['trips']:
		adjustments_this=False
		uid=int(trip['uid'])
		numWP=len(trip['waypoints'])
		if not uid in AP+AP_NC:
			for wp_index in range(2,numWP-1):
				wp=int(trip['waypoints'][wp_index])
				if (uid,wp) in WP_DEV_ampl.members():
					wp_prev=int(trip['waypoints'][wp_index-1])
					arrival=var_t_earliest[uid,wp].value()
					arrival_prev=var_t_earliest[uid,wp_prev].value()
					travelling_time=arrival-arrival_prev
					speed=dist[wp_prev,wp]/travelling_time
					percentage_change=speed/v_default
					if percentage_change<=1-0.0000001: # deceleration
						if adjustments_this==False: 
							adjustments_this=True
							num_tripsSpeedAdjusted+=1
						if nDrift>0 and uid==drifted[0]:
							speedAdjustment_driftedTrip+=1
						total_speedAdjustments+=1
						percentage_change=(1-percentage_change)*100
						for percent in range(len(decelerations)):
							if percentage_change<=percent+1.000001: 
								decelerations[percent]+=1
								break
					elif percentage_change>=1+0.0000001: # acceleration
						if adjustments_this==False: 
							adjustments_this=True
							num_tripsSpeedAdjusted+=1
						if nDrift>0 and uid==drifted[0]:
							speedAdjustment_driftedTrip+=1
						total_speedAdjustments+=1
						percentage_change=(percentage_change-1)*100
						for percent in range(len(accelerations)):
							if percentage_change<=percent+1.000001: 
								accelerations[percent]+=1
								break
	#Print summary to console
	if verbose:
		sys.stdout.write('\n -------- Skyline network %s -------------'%(input_file))
		sys.stdout.write('\n nodes=%i, fatos=%i, trips=%i, inflight=%i, nDelay=%i, nDrift=%i'%(nNodes,nFatos,len(A),len(inflight),nDelay,nDrift))
		sys.stdout.write('\n optimal value: %f'%objVal)
		sys.stdout.write('\texit status:%s'%status) 
		print('\nModified trajectories (departures):')
		print_solution(ampl,0,isLocal)
		#print_solution(ampl,1,isLocal)
		# print('\nModified trajectories (1st node at cruise level):')
		print('\nModified trajectories (inner junctions):')
		print_solution(ampl,-100,isLocal)
		print('\nModified trajectories (last node at cruise level):')
		print_solution(ampl,-2,isLocal)
		if nIntru>0: 
			print('\t delay priority traffic %i: %i'%(AP[0],totalDelay_Priority))
		print('touched trips (and intruder, if any): ',list_touched)
		print('touched trips (w.r.t. rounded schedules): ',list_touchedRounded)
		sys.stdout.write('delayed trips: %i \t counted total deviation: %f\n'%(num_delayedDeparture,totalDev_nonPriority))
		sys.stdout.write('Number of speed adjustments: %i\n'%total_speedAdjustments)
		print('Number of accelerations...')
		for percent in range(len(accelerations)):
			sys.stdout.write('\t in (%i,%i]: %i\n'%(percent,percent+1,accelerations[percent]))
		print('Number of decelerations...')
		for percent in range(len(decelerations)):
			sys.stdout.write('\t in (%i,%i]: %i\n'%(percent,percent+1,decelerations[percent]))
	#  _ampl_time and _total_solve_time
	df_time1=ampl.getData('_ampl_time')
	df_time2=ampl.getData('_total_solve_time')
	list_time1=df_time1.toList()
	list_time2=df_time2.toList()
	time1=list_time1[0]
	time2=list_time2[0]
	# Case of global TD: generate R code to plot solution
	if isLocal==False and list_touched:
		file_split=input_file.rsplit('/',1)
		file_split=re.split('[\.]', file_split[len(file_split)-1])
	 	R_plot(list_touched,'tEarliest','tLatest',file_split[0]+"-"+str(random_seed)+"-"+str(nDelay)+str(nDrift)+str(nIntru)+str(nNonColIntru)+"-plot.R")
	 	R_plot(list_touched,'new_tEarliest','new_tLatest',file_split[0]+"-"+str(random_seed)+"-"+str(nDelay)+str(nDrift)+str(nIntru)+str(nNonColIntru)+"-plot.R")
	# Output summary to file
	original=sys.stdout 
	sys.stdout=open(output_file,'a')
	sys.stdout.write('\t%.4f\t%.4f\t%.4f'%(objVal,time1,time2))
	sys.stdout.write('\t%s'%status)
	sys.stdout.write('\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%i\t%i'%(latestArrival_new,
								totalDev_nonPriority,totalDelay_Priority,
								totalDelay_departure,totalDelay_destination,
								num_delayedDeparture,num_delayedDestination))
	sys.stdout.write('\t%.4f\t%i\t%.4f\t%i\t%.4f\t%i\t%.4f\t%i'%(
								max_dev,wp_max_dev,max_totalDev_at_wp,wp_maxTotalDev,
								max_delay_o, wp_max_delay_o,max_delay_d, wp_max_delay_d))
	sys.stdout.write('\t%i\t%i\t%.4f\t%.4f\t%.4f\t%i'%(num_touchedTrips,num_touchedRounded,
								rounded_totalDev_nonPriority,rounded_totalDelay_destination,
								rounded_max_dev,wp_rounded_max_dev))
	sys.stdout.write('\t%i\t%i\t%i'%(total_speedAdjustments,speedAdjustment_driftedTrip,num_tripsSpeedAdjusted))
	for percent in range(len(accelerations)):
		sys.stdout.write('\t%i'%(accelerations[percent]))
	for percent in range(len(decelerations)):
		sys.stdout.write('\t%i'%(decelerations[percent]))
	sys.stdout=original
#------------ end function

##############################################################################
#  					 2. Read instance data					 				 #
############################################################################## 
read_SD_schedule(input_file)
if verbose:
	print("** isFATO: ",isFATO)
	print("** Edge list: ",edges)
	print("** Num nodes= ",nNodes)
	print("** Num FATOs= ",nFatos)
	print("** Num edges= ",nEdges)
	for trip in schedule['trips']:
		print(trip['uid'])
		print(trip['waypoints'])
		print(trip['tEarliest'])
		print(trip['tLatest'])
		print('-----------------------')


##############################################################################
#  		    3. Scenario generation 		 		      						 #
############################################################################## 
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
if nDrift+nDelay>0:
	disruptions_scenario1(nDrift,nDelay,int(math.floor(latestArrival_nominal))) 
	if verbose:
		print('timeTD',timeTD)
		print('wp of the drift',wpDrift)
		print('delayed trips',delayed)
		print('drifted trips',drifted)
		print('inflight trips',inflight)
		print('ground trips',ground)
#--------------3b. Scenario 2: collaborative intruder
o_intru=-1
d_intru=-1
if nIntru>0:
	intruder_scenario2(int(math.floor(latestArrival_nominal))) 
	if verbose:
		print('timeTD',timeTD)
		print('priority trips',AP)
		print('inflight trips',inflight)
		print('ground trips',ground)
		for trip in schedule['trips']:
			if trip['uid']==AP[0]:
				print('waypoints',trip['waypoints'])
				print('tEarliest',trip['tEarliest'])

#--------------3c. Scenario 3: non-collaborative intruder
if nNonColIntru>0:
	intruder_scenario3(int(math.floor(latestArrival_nominal))) 
	if verbose:
		print('timeTD',timeTD)
		print('non-collaborative intruder',AP_NC)
		print('inflight trips',inflight)
		print('ground trips',ground)
		for trip in schedule['trips']:
			if trip['uid']==AP_NC[0]:
				print('waypoints',trip['waypoints'])
				print('tEarliest',trip['tEarliest'])

##############################################################################
#  		 4. 	 Tactical Deconfliction Model	 							 #
############################################################################## 
ampl = AMPL()
ampl.read('../../implementation/tactical-skylane.mod')
load_AMPL_data(ampl)
# If drift in scenario 1: fix the varibles of the drift and drop corresponding constraints
if nDrift >0: 
	include_drift(ampl)
if verbose:
	print('\n###########################################')
	print('###     Executing **global** TD model   ###')
	print('###########################################')
set_AMPL_model(ampl,set(A)-set(AP)-set(AP_NC)) 
ampl.solve()
inConflict_trips=[]
conflict_generators=[]
instance_info_output(ampl)
obj=ampl.getObjective('Dev')
status=obj.result()
if status=='infeasible':
	if verbose: print('Infeasible problem')
	original=sys.stdout 
	sys.stdout=open(output_file,'a')
	sys.stdout.write('\t infeasible')
	sys.stdout=original
else:
	solution_postprocess_and_output(ampl,False)
ampl.close()

##############################################################################
#  	 5. Tactical Deconfliction only with inConflict trips	 				 #
############################################################################## 
if compareWithLocal:
	ampl = AMPL()
	ampl.read('../../implementation/tactical-skylane.mod')
	load_AMPL_data(ampl)
	# If drift in scenario 1: fix the varibles of the drift and drop corresponding constraints
	if nDrift >0: 
		include_drift(ampl)
	if verbose:
		print('\n###########################################')
		print('###     Executing **local** TD model   ###')
		print('###########################################')
	set_AMPL_model(ampl,set(list(inConflict_trips)+conflict_generators)-set(AP)-set(AP_NC)) 
	ampl.solve()
	obj=ampl.getObjective('Dev')
	status=obj.result()
	original=sys.stdout 
	sys.stdout=open(output_file,'a')
	if status=='infeasible':
		sys.stdout.write('\t || infeasible')
		sys.stdout=original
		if verbose: print('Infeasible problem')
	else:
		sys.stdout.write('\t ||')
		sys.stdout=original
		solution_postprocess_and_output(ampl,True)