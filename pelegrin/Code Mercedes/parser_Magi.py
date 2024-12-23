
import math
import json
import random
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import sys
import re
import copy

input_file = sys.argv[1]
random_seed = sys.argv[2]
output = ""
if input_file == "1":
    input_file = "/home/magi/UAMdeconflictionMasterThesis/pelegrin/Code Mercedes/geoGebra/net.txt"
    output_file = "/home/magi/UAMdeconflictionMasterThesis/modelli/data/mercedesSD/grid" + random_seed + ".txt"
elif input_file == "2": 
    input_file = "/home/magi/UAMdeconflictionMasterThesis/pelegrin/Code Mercedes/geoGebra/net2.txt"
    output_file = "/home/magi/UAMdeconflictionMasterThesis/modelli/data/mercedesSD/airport" + random_seed + ".txt"
elif input_file == "3": 
    input_file = "/home/magi/UAMdeconflictionMasterThesis/pelegrin/Code Mercedes/geoGebra/net3.txt"
    output_file = "/home/magi/UAMdeconflictionMasterThesis/modelli/data/mercedesSD/metroplex" + random_seed + ".txt"
verbose = float("0")  # draw net? print outputs?
random.seed(random_seed)
# deconf parameters:
D = 0.5
tErr = 1
v_default = 2.5
v_climb = 1.5
# schedule parameters:
numBatches = int(5)
tripsPerBatch = int("20")
timeBatch = float("10")
min_trip_duration = 7
max_trip_duration = 25

###################################################################
#        1. Read network from psTricks input file   			  #
###################################################################
nNodes = 0
nEdges = 0
nFatos = 0
nodesCoord = {}
nodeType = (
    {}
)  # 0=take off junc (insertion); 1=landing junc (split); 2= inner junc (sync); 3=take off FATO; 4=landing FATO
isFATO = {}
edges = []
skylanes = []

with open(input_file) as openfileobject:
    for line in openfileobject:
        # 1. Nodes
        if line.startswith("\\psdots"):
            # \psdots[dotsize=7pt 0,dotstyle=square*,dotangle=45,linecolor=blue](7.5,9)

            # (x,y)-coordinates: the units are travelling times in min, not miles
            vector_coord = re.search("[(](.*?)[)]", line)
            vector_coord = re.split("[(),]+", vector_coord.group(0))
            vector_coord = list(filter(None, vector_coord))
            vector_coord = np.array(vector_coord)
            vector_coord.astype(np.double)
            # until now checked and working

            # z-coordinate: FATO or cruise?
            level = re.search("dotstyle=(.*?),", line)
            if level.group(0) == "dotstyle=*,":  # cruise
                isFATO[nNodes] = False
                # nodesCoord[nNodes]=[float(vector_coord[0]),float(vector_coord[1]),0.284091] # cruise level set to 0.284091 miles=1500 ft
                nodesCoord[nNodes] = [
                    float(vector_coord[0]),
                    float(vector_coord[1]),
                    1,
                ]  # cruise level set to 1 min
            else:  # FATO
                isFATO[nNodes] = True
                nFatos = nFatos + 1
                # height=random.uniform(30,200) #in feets, simulates buildings height
                # height=height*0.000189394
                # nodesCoord[nNodes]=[float(vector_coord[0]),float(vector_coord[1]),height]
                nodesCoord[nNodes] = [
                    float(vector_coord[0]),
                    float(vector_coord[1]),
                    0,
                ]  # FATO level set to 0
            # node type
            color = re.search("linecolor=(.*?)\]", line).group(1)
            if color == "qqzzqq":  # green, inner junc, code 2
                nodeType[nNodes] = 2
            elif color == "ffvvqq":  # orange, inner break point within edge, code 5
                nodeType[nNodes] = 5
            elif color == "ccqqqq":  # red, take off, code 0 or 3
                if isFATO[nNodes]:
                    nodeType[nNodes] = 3
                else:
                    nodeType[nNodes] = 0
            elif color == "blue":  # blue, landing, code 1 or 4
                if isFATO[nNodes]:
                    nodeType[nNodes] = 4
                else:
                    nodeType[nNodes] = 1
            else:
                print("Warning: Unrecognized color")
            print(nNodes, nodesCoord[nNodes])
            nNodes = nNodes + 1
        # 2. Edges and skylanes
        elif line.startswith("\\psline"):
            vector_coord = re.search("[(](.*?)[)][(](.*?)[)]", line)
            vector_coord = re.split("[(),]+", vector_coord.group(0))
            vector_coord = list(filter(None, vector_coord))
            vector_coord = np.array(vector_coord)
            vector_coord.astype(np.double)

            if line.startswith("\\psline[linewidth=2pt]{->}"):  # edge
                found_u = False
                found_v = False
                u = 0
                v = 0
                for node in range(nNodes):
                    if (
                        found_u == False
                        and math.isclose(nodesCoord[node][0], float(vector_coord[0]))
                        and math.isclose(nodesCoord[node][1], float(vector_coord[1]))
                    ):
                        found_u = True
                        u = node
                    elif (
                        found_v == False
                        and math.isclose(nodesCoord[node][0], float(vector_coord[2]))
                        and math.isclose(nodesCoord[node][1], float(vector_coord[3]))
                    ):
                        found_v = True
                        v = node
                if (found_u == False) or (found_v == False):
                    print("error, u or v not found")
                    print(line)
                if isFATO[u] == True or isFATO[v] == True:
                    print("edge FATO", u, v)
                    if nodeType[v] == 3:
                        edges.append((v, u))
                    elif nodeType[u] == 4:
                        edges.append((v, u))
                    else:
                        edges.append((u, v))
                else:
                    print("edge", u, v)
                    edges.append((u, v))
                nEdges = nEdges + 1
            else:  # skylane
                skylanes.append(
                    (
                        float(vector_coord[0]),
                        float(vector_coord[1]),
                        float(vector_coord[2]),
                        float(vector_coord[3]),
                    )
                )

if verbose:
    print("** Nodes coordinates: ", nodesCoord)
    print("** isFATO: ", isFATO)
    print("** Edge list: ", edges)
    print("** Num nodes= ", nNodes)
    print("** Num FATOs= ", nFatos)
    print("** Num edges= ", nEdges)





###################################################################
#            2. Create graph from read data   					  #
###################################################################
# ----------- Create graph
G = nx.DiGraph()
dist = {}
for i in range(nEdges):
    u = edges[i][0]
    v = edges[i][1]
    x = nodesCoord[u][0] - nodesCoord[v][0]
    y = nodesCoord[u][1] - nodesCoord[v][1]
    z = nodesCoord[u][2] - nodesCoord[v][2]
    # d=math.sqrt(x*x+y*y+z*z) # 1st version, calculate Euclidean distance
    # G.add_edge(u,v,dist=d)
    print(u, v)
    G.add_edge(u, v, dist=1)  # 2nd version, fix distance to 1 minute travelling time
    if not isFATO[u] and not isFATO[v]:
        # dist[u,v]=d*v_default # 1st version, distances of drawing were traveling distances
        dist[u, v] = v_default  # 2nd version
    else:
        # dist[u,v]=d # this line is for arbitrary climbing times (1st version)
        # dist[u,v]=d*v_climb # this yields climbing times of 1 minute if d=1 (d is travelling time)
        dist[u, v] = v_climb  # 2nd version


# ----------- Calculatetrip_durat angles of skylane junctions at cruise level
angleJunc = {}
G2 = G.to_undirected()
for i in range(nNodes):
    if nodeType[i] != 3 and nodeType[i] != 4:  # node is not a FATO
        for j in G2.neighbors(i):
            for k in G2.neighbors(i):
                if (
                    j < k
                    and (nodeType[j] != 3 and nodeType[j] != 4)
                    and (nodeType[k] != 3 and nodeType[k] != 4)
                ):
                    v1_ij = nodesCoord[i][0] - nodesCoord[j][0]
                    v2_ij = nodesCoord[i][1] - nodesCoord[j][1]
                    v1_ik = nodesCoord[i][0] - nodesCoord[k][0]
                    v2_ik = nodesCoord[i][1] - nodesCoord[k][1]
                    v_ij = [v1_ij, v2_ij]
                    v_ik = [v1_ik, v2_ik]
                    unit_v_ij = v_ij / np.linalg.norm(v_ij)
                    unit_v_ik = v_ik / np.linalg.norm(v_ik)
                    dot_product = np.dot(unit_v_ij, unit_v_ik)
                    if math.isclose(dot_product, 1):
                        dot_product = 1.
                    elif math.isclose(dot_product, -1):
                        dot_product = -1.
                    angle = np.arccos(dot_product)
                    angleJunc[i, j, k] = angle
                    angleJunc[i, k, j] = angle

print(angleJunc)
# ----------Draw skylane network
print(G.nodes, G.edges)
if verbose:
    coord_planar = {}
    coord_junc = {}
    coord_ports = {}
    color_ports = []
    ports_list = []
    junc_list = []
    label_map = {}
    for u in range(nNodes):
        if nodeType[u] == 3 or nodeType[u] == 4:  # node is skyport
            coord_ports[u] = [nodesCoord[u][0], nodesCoord[u][1] + 0.5]
            coord_planar[u] = coord_ports[u]
            ports_list.append(u)
            color_ports.append("gray")
            label_map[u] = u
        elif nodeType[u] == 0 or nodeType[u] == 1:  # node is skyport junction
            coord_ports[u] = [nodesCoord[u][0], nodesCoord[u][1]]
            coord_planar[u] = coord_ports[u]
            ports_list.append(u)
            color_ports.append("white")
            label_map[u] = u
        else:  # node is inner junction
            coord_junc[u] = [nodesCoord[u][0], nodesCoord[u][1]]
            coord_planar[u] = coord_junc[u]
            junc_list.append(u)
            label_map[u] = u

    nx.draw_networkx_nodes(
        G,
        pos=coord_ports,
        node_size=350,
        node_shape="d",
        nodelist=ports_list,
        node_color=color_ports,
    )
    nodes = nx.draw_networkx_nodes(
        G,
        pos=coord_junc,
        node_size=350,
        node_shape="o",
        node_color="white",
        nodelist=junc_list,
    )
    if nodes:
        nodes.set_edgecolor("black")
    nx.draw_networkx_edges(G, coord_planar, arrows=True)
    nx.draw_networkx_labels(G, coord_planar, labels=label_map)

    ax = plt.gca()
    ax.collections[0].set_edgecolor("#000000")

    plt.axis("off")
    plt.show()

##############################################################################
#  					 3. Generate set of request 							 #
##############################################################################
nSkyports = nFatos / 2
takeoffFATO_list = []
landingFATO_list = []
tuples_nodePath = []
tuples_linkPath = []
for i in range(nNodes):
    if nodeType[i] == 3:
        takeoffFATO_list.append(i)
    if nodeType[i] == 4:
        landingFATO_list.append(i)
schedule = json.loads("""{"trips":[]}""")  # json list with trips
orig_list = []
dest_list = []
for batch in range(0, numBatches):
    orig_index = []
    for i in range(0, tripsPerBatch):
        orig_index.append(random.randint(0, nSkyports - 1))
    tuples_nodePath_batch = []
    tuples_linkPath_batch = []
    orig_list_batch = {}
    dest_list_batch = {}
    for trip in range(0, tripsPerBatch):
        o = takeoffFATO_list[orig_index[trip]]
        d = o
        path = []
        trip_duration = max_trip_duration + 1
        while (
            d == o
            or trip_duration < min_trip_duration
            or trip_duration > max_trip_duration
        ):
            dest_index = random.randint(0, nSkyports - 1)
            if dest_index != orig_index[trip]:
                d = landingFATO_list[dest_index]
                path = nx.shortest_path(
                    G, o, d, weight="dist"
                )  # network weight is the distance
                path_distance = nx.shortest_path_length(G, o, d, weight="dist")
                trip_duration = path_distance
            else:
                d = o
        uid = batch * tripsPerBatch + trip
        orig_list_batch[uid] = o
        dest_list_batch[uid] = d
        schedule["trips"].append({"uid": uid, "waypoints": path, "batch": batch})
        for wp in path:
            tuples_nodePath_batch.append((uid, wp))
        for i in range(len(path) - 1):
            tuples_linkPath_batch.append((uid, path[i], path[i + 1]))
    tuples_nodePath.append(tuples_nodePath_batch)
    tuples_linkPath.append(tuples_linkPath_batch)
    orig_list.append(orig_list_batch)
    dest_list.append(dest_list_batch)
nTrips = tripsPerBatch * numBatches
if verbose:
    print(schedule)

# Commented out IPython magic to ensure Python compatibility.
# Install dependencies
# %pip install -q amplpy

"""Put your own license_uuid where amol is defined"""

# Commented out IPython magic to ensure Python compatibility.
##############################################################################
# 			  4. Do strategic deconf with AMPL model						 #
##############################################################################
from amplpy import AMPL
from amplpy import DataFrame


pending_from_previous_batch = []  # trips from previous batch that were not scheduled
inflight_from_previous_batch = []  # trips from previous batch that are still flying

scheduled_t_ear = {}  # store SD solution: earliest time for trip uid at waypoint wp
scheduled_t_lat = {}  # store SD solution: latest time for trip uid at waypoint wp

pending_from_previous_batch = []
inflight_from_previous_batch = []

sepCruise_slacks = (
    []
)  # keep the minimum of the slacks of separation constraints of the SD model for each batch
sepClimb_slacks = []

for batch in range(0, numBatches):
    # 4a ...Load AMPL model and....
    ampl = AMPL()  # instantiate AMPL object and register magics
    ampl.read("/home/magi/UAMdeconflictionMasterThesis/pelegrin/Code Mercedes/strategic-skylane.mod")
    # .... pass trip requests to AMPL data for SD
    A = (
        list(range(batch * tripsPerBatch, batch * tripsPerBatch + tripsPerBatch))
        + pending_from_previous_batch
        + inflight_from_previous_batch
    )
    A_ampl = ampl.getSet("A")
    A_ampl.setValues(A)
    AP = ampl.getSet("AP")
    AP.setValues(pending_from_previous_batch)
    NODES = ampl.getSet("NODES")
    NODES.setValues(range(nNodes))
    FATOS = ampl.getSet("FATOS")
    FATOS.setValues(takeoffFATO_list + landingFATO_list)
    LINKS = ampl.getSet("LINKS")
    LINKS.setValues(edges)
    D_ampl = ampl.getParameter("D")
    terr_ampl = ampl.getParameter("terr")
    tini_ampl = ampl.getParameter("tini")
    vcruise_ampl = ampl.getParameter("vcruise")
    vclimb_ampl = ampl.getParameter("vclimb")
    horizon_ampl = ampl.getParameter("horizon")
    D_ampl.set(D)
    terr_ampl.set(tErr)
    tini_ampl.set(batch * timeBatch)
    vcruise_ampl.set(v_default)
    vclimb_ampl.set(v_climb)
    horizon_ampl.set(timeBatch)
    dist_ampl = ampl.getParameter("dist")
    angleJunc_ampl = ampl.getParameter("angleJunction")
    dist_ampl.setValues(dist)
    angleJunc_ampl.setValues(angleJunc)
    # update data by considering also trips from previous batch
    if (
        pending_from_previous_batch + inflight_from_previous_batch
    ):  # at least one of these lists is not empty
        previous_orig = dict(
            (k, orig_list[batch - 1][k])
            for k in (pending_from_previous_batch + inflight_from_previous_batch)
            if k in orig_list[batch - 1]
        )
        orig_list[batch].update(previous_orig)
        previous_dest = dict(
            (k, dest_list[batch - 1][k])
            for k in (pending_from_previous_batch + inflight_from_previous_batch)
            if k in dest_list[batch - 1]
        )
        dest_list[batch].update(previous_dest)
        for uid in pending_from_previous_batch + inflight_from_previous_batch:
            for tuple in tuples_nodePath[batch - 1]:
                if tuple[0] == uid:
                    tuples_nodePath[batch].append(tuple)
            for tuple in tuples_linkPath[batch - 1]:
                if tuple[0] == uid:
                    tuples_linkPath[batch].append(tuple)
    LINKSPATH = ampl.getSet("LINKSPATH")
    LINKSPATH.setValues(tuples_linkPath[batch])
    NODESPATH = ampl.getSet("NODESPATH")
    NODESPATH.setValues(tuples_nodePath[batch])
    orig_ampl = ampl.getParameter("orig")
    # dest_ampl=ampl.getParameter('dest')
    orig_ampl.setValues(orig_list[batch])
    # dest_ampl.setValues(dest_list[batch])

    # 4b ...Fix variables corresponding to in-flight trips, drop corresponding constraints
    var_t_earliest = ampl.getVariable("tmin")
    var_t_latest = ampl.getVariable("tmax")
    var_t_earliest_orig = ampl.getVariable("tmin_orig")
    var_t_latest_orig = ampl.getVariable("tmax_orig")
    var_scheduled = ampl.getVariable("scheduled")
    for uid in inflight_from_previous_batch:
        var_scheduled[uid].fix(1)
        var_t_earliest_orig[uid].fix(scheduled_t_ear[uid, orig_list[batch][uid]])
        var_t_latest_orig[uid].fix(scheduled_t_lat[uid, orig_list[batch][uid]])
        const1 = ampl.getConstraint("iniTimeBatch")
        const2 = ampl.getConstraint("endTimeBatch")
        const3 = ampl.getConstraint("tmin_origin")
        const4 = ampl.getConstraint("tmax_origin")
        const5 = ampl.getConstraint("width_origin")
        const1[uid].drop()
        const2[uid].drop()
        const3[uid].drop()
        const4[uid].drop()
        const5[uid].drop()
    for uid, wp in tuples_nodePath[batch]:
        if uid in inflight_from_previous_batch:
            var_t_earliest[uid, wp].fix(scheduled_t_ear[uid, wp])
            var_t_latest[uid, wp].fix(scheduled_t_lat[uid, wp])
    for uid, wp1, wp2 in tuples_linkPath[batch]:
        if uid in inflight_from_previous_batch and (
            nodeType[wp1] != 3 and nodeType[wp2] != 4
        ):
            const8 = ampl.getConstraint("tmin_cruise")
            const9 = ampl.getConstraint("tmax_cruise")
            const8[uid, wp1, wp2].drop()
            const9[uid, wp1, wp2].drop()
        elif uid in inflight_from_previous_batch and (
            nodeType[wp1] == 3 or nodeType[wp2] == 4
        ):
            const10 = ampl.getConstraint("tmin_climb")
            const11 = ampl.getConstraint("tmax_climb")
            const10[uid, wp1, wp2].drop()
            const11[uid, wp1, wp2].drop()
    # 4c ...Call AMPL SD model and obtain solution-- if dropped trips, put in next batch
    ampl.setOption(
        "presolve_eps", 0.0000000000136
    )  # this helps to avoid issues with separation constraints of 2 fixed trips from previous batch
    ampl.setOption("solver_msg", 0)
    ampl.option["solver"] = "cplex"
    goal1 = ampl.getObjective("ScheduledTrips")
    goal2 = ampl.getObjective("MaxDeparture")
    var_numScheduled_nonpriority = ampl.getVariable("numScheduled_nonpriority")
    var_numScheduled_priority = ampl.getVariable("numScheduled_priority")
    # 4c.1: first goal: max number of scheduled trips
    goal2.drop()
    original = sys.stdout  # to avoid output messages from solve()
    sys.stdout = open("/dev/null", "w")
    ampl.solve()
    sys.stdout = original
    # df2 = ampl.getData("{(i,m,h) in LINKSPATH: h in FATOS}  texact[i,m]+dist[m,h]/vclimb")
    # df1 = ampl.getData("{(i,m) in NODESPATH: m in FATOS}  texact[i,m]")
    # print(df2)
    # print(df1)
    # sys.exit()
    # if verbose, print solution
    if verbose:
        print("-------------new batch-------------")
        print("goal 1..........")
        sys.stdout.write("optimal value: %f \n" % goal1.value())
        sys.stdout.write(
            "new scheduled trips: %f \n"
             % (
                int(var_numScheduled_nonpriority.value())
                - len(inflight_from_previous_batch)
            )
        )
        sys.stdout.write(
            "old scheduled trips: %f \n" % var_numScheduled_priority.value()
        )
        sys.stdout.write(
            "not scheduled trips: %f \n"
             % (
                len(A)
                - var_numScheduled_priority.value()
                - var_numScheduled_nonpriority.value()
            )
        )
        df_t_earliest = var_t_earliest.getValues()
        df_t_latest = var_t_latest.getValues()
        df_scheduled = var_scheduled.getValues()
        # print(df_t_earliest)
        # print(df_t_latest)
        # print(df_scheduled)
    # fix number of schedules trips (1st goal)
    var_numScheduled_nonpriority.fix(var_numScheduled_nonpriority.value())
    var_numScheduled_priority.fix(var_numScheduled_priority.value())

    # 4c.2: second goal: min max time departure
    goal1.drop()
    goal2.restore()
    original = sys.stdout  # to avoid output messages from solve()
    sys.stdout = open("/dev/null", "w")
    ampl.solve()
    sys.stdout = original
    # print
    if verbose:
        var_maxDeparture = ampl.getVariable("maxDeparture")
        df_t_earliest = var_t_earliest.getValues()
        df_t_latest = var_t_latest.getValues()
        df_scheduled = var_scheduled.getValues()
        print("goal 2..........")
        # print(df_t_earliest)
        # print(df_t_latest)
        # print(df_scheduled)
        sys.stdout.write("optimal value: %f \n" % goal2.value())
        sys.stdout.write(
            "new scheduled trips: %f \n"
             % (
                int(var_numScheduled_nonpriority.value())
                - len(inflight_from_previous_batch)
            )
        )
        sys.stdout.write(
            "old scheduled trips: %f \n" % var_numScheduled_priority.value()
        )
        sys.stdout.write(
            "not scheduled trips: %f \n"
             % (
                len(A)
                - var_numScheduled_priority.value()
                - var_numScheduled_nonpriority.value()
            )
        )
        # print(var_maxDeparture.value())
    # store schedules
    for index, ampl_var in var_t_earliest:
        uid, wp = index
        if var_scheduled[uid].value() == 1:
            scheduled_t_ear[uid, wp] = ampl_var.value()
    for index, ampl_var in var_t_latest:
        uid, wp = index
        if var_scheduled[uid].value() == 1:
            scheduled_t_lat[uid, wp] = ampl_var.value()
    # save slack of separation constraints
    set_cross = ampl.getSet("CROSS_CONF")  # (i,j,m,h_i,l_i,h_j,l_j) in CROSS_CONF
    const_crossA = ampl.getConstraint("sepCross_A")
    const_crossB = ampl.getConstraint("sepCross_B")
    const_crossC = ampl.getConstraint("sepCross_C")
    slack_crossA = float("inf")
    slack_crossB = float("inf")
    slack_crossC = float("inf")
    for i, j, m, h_i, l_i, h_j, l_j in set_cross.getValues():
        slack = const_crossA[i, j, m, h_i, l_i, h_j, l_j].slack()
        if slack < slack_crossA:
            slack_crossA = slack
        slack = const_crossB[i, j, m, h_i, l_i, h_j, l_j].slack()
        if slack < slack_crossB:
            slack_crossB = slack
        slack = const_crossC[i, j, m, h_i, l_i, h_j, l_j].slack()
        if slack < slack_crossC:
            slack_crossC = slack
    set_merge = ampl.getSet("MERGE_CONF")  # (i,j,m,h_i,h_j,l) in MERGE_CONF
    const_merge = ampl.getConstraint("sepMerge")
    slack_merge = float("inf")
    for i, j, m, h_i, h_j, l in set_merge.getValues():
        slack = const_merge[i, j, m, h_i, h_j, l].slack()
        if slack < slack_merge:
            slack_merge = slack
    set_split = ampl.getSet("SPLIT_CONF")  # (i,j,m,h,l_i,l_j) in SPLIT_CONF
    const_split = ampl.getConstraint("sepSplit")
    slack_split = float("inf")
    for i, j, m, h, l_i, l_j in set_split.getValues():
        slack = const_split[i, j, m, h, l_i, l_j].slack()
        if slack < slack_split:
            slack_split = slack
    set_merge_land = ampl.getSet("MERGE_LAND")  # (i,j,m,h_i,h_j) in MERGE_LAND
    const_mergeLand = ampl.getConstraint("sepMerge_land")
    slack_mergeLand = float("inf")
    for i, j, m, h_i, h_j in set_merge_land.getValues():
        slack = const_mergeLand[i, j, m, h_i, h_j].slack()
        if slack < slack_mergeLand:
            slack_mergeLand = slack
    set_split_takeoff = ampl.getSet("SPLIT_TAKEOFF")  # (i,j,m,l_i,l_j) in SPLIT_TAKEOFF
    const_splitTakeoff = ampl.getConstraint("sepSplit_takeoff")
    slack_splitTakeoff = float("inf")
    for i, j, m, l_i, l_j in set_split_takeoff.getValues():
        slack = const_splitTakeoff[i, j, m, l_i, l_j].slack()
        if slack < slack_splitTakeoff:
            slack_splitTakeoff = slack
    set_trail_land = ampl.getSet("TRAIL_LAND")  # (i,j,m,h) in TRAIL_LAND
    const_trailLand = ampl.getConstraint("sepTrail_land")
    slack_trailLand = float("inf")
    for i, j, m, h in set_trail_land.getValues():
        slack = const_trailLand[i, j, m, h].slack()
        if slack < slack_trailLand:
            slack_trailLand = slack
    set_trail_takeoff = ampl.getSet("TRAIL_TAKEOFF")  # (i,j,m,h) in TRAIL_TAKEOFF
    const_trailTakeoff = ampl.getConstraint("sepTrail_takeoff")
    slack_trailTakeoff = float("inf")
    for i, j, m, h in set_trail_takeoff.getValues():
        slack = const_trailTakeoff[i, j, m, h].slack()
        if slack < slack_trailTakeoff:
            slack_trailTakeoff = slack
    set_trail = ampl.getSet("TRAIL_CRUISE")  # (i,j,m,h) in TRAIL_CRUISE
    const_trailIni = ampl.getConstraint("sepTrail_ini")
    const_trailEnd = ampl.getConstraint("sepTrail_end")
    slack_trailIni = float("inf")
    slack_trailEnd = float("inf")
    for i, j, m, h in set_trail.getValues():
        slack = const_trailIni[i, j, m, h].slack()
        if slack < slack_trailIni:
            slack_trailIni = slack
        slack = const_trailEnd[i, j, m, h].slack()
        if slack < slack_trailEnd:
            slack_trailEnd = slack
    if verbose:
        print(
            slack_crossA,
            slack_crossB,
            slack_crossC,
            slack_merge,
            slack_split,
            slack_trailIni,
            slack_trailEnd,
        )  # cruise slack
        print(
            slack_mergeLand, slack_splitTakeoff, slack_trailLand, slack_trailTakeoff
        )  # climb slack
    sepClimb_slacks.append(
        min(slack_mergeLand, slack_splitTakeoff, slack_trailLand, slack_trailTakeoff)
    )
    sepCruise_slacks.append(
        min(
            slack_crossA,
            slack_crossB,
            slack_crossC,
            slack_merge,
            slack_split,
            slack_trailIni,
            slack_trailEnd,
        )
    )
    # update variables for next iteration
    pending_from_previous_batch = []
    inflight_from_previous_batch = []
    for uid in A:
        if var_scheduled[uid].value() == 1:
            dest = dest_list[batch][uid]
            if scheduled_t_lat[uid, dest] > (batch + 1) * timeBatch:
                inflight_from_previous_batch.append(uid)
        else:
            if verbose:
                print(uid, "not scheduled")
            pending_from_previous_batch.append(uid)
if verbose:
    print("Separation slacks per batch:")
    print("climb: ", sepClimb_slacks)
    print("cruise: ", sepCruise_slacks)

##############################################################################
# 	  5. Postprocessing: round intermediate arrival times					 #
##############################################################################
totalScheduled = 0
for trip in schedule["trips"]:
    uid = trip["uid"]
    isScheduled = False
    for wp_index in range(1, len(trip["waypoints"]) - 1):
        wp = trip["waypoints"][wp_index]
        if (uid, wp) in scheduled_t_ear:
            if isScheduled == False:
                isScheduled = True
                totalScheduled = totalScheduled + 1
            # scheduled_t_ear[uid,wp]=math.ceil(scheduled_t_ear[uid,wp]*10)/10
            # scheduled_t_lat[uid,wp]=math.floor(scheduled_t_lat[uid,wp]*10)/10

# Commented out IPython magic to ensure Python compatibility.
##############################################################################
# 	   6. Output scenario + scheduled trips after SD 			        	 #
##############################################################################
file = open(output_file, "w")
# network details
file.write("nNodes: %i\n" % nNodes)
file.write("nFatos: %i\n" % nFatos)
file.write("node list:\n")
for i in range(nNodes):
    file.write(
        "\t%i %i %f %f\n" % (i, nodeType[i], nodesCoord[i][0], nodesCoord[i][1])
    )
file.write("nEdges: %i\n" % nEdges)
file.write("edge list:\n")
for i in range(nEdges):
    file.write(
        "\t%i %i %f\n" % (edges[i][0], edges[i][1], dist[edges[i][0], edges[i][1]])
    )
file.write("nSkylanes: %i\n" % len(skylanes))
file.write("skylanes list:\n")
for i in range(len(skylanes)):
    file.write(
        "\t%f %f %f %f\n"
         % (skylanes[i][0], skylanes[i][1], skylanes[i][2], skylanes[i][3])
    )
file.write("nAngles: %i\n" % len(angleJunc))
file.write("angle junction list:\n")
for i, j, k in angleJunc:
    file.write("\t%i %i %i %f\n" % (i, j, k, angleJunc[i, j, k]))
# deconf details
file.write("D: %f\n" % D)
file.write("v_default: %f\n" % v_default)
file.write("v_climb: %f\n" % v_climb)
# schedule details
file.write("numBatches: %i\n" % numBatches)
file.write("tripsPerBatch: %i\n" % tripsPerBatch)
file.write("timeBatch: %i\n" % timeBatch)
file.write("numTrips: %i\n" % totalScheduled)
for trip in schedule["trips"]:
    uid = trip["uid"]
    if (uid, trip["waypoints"][0]) in scheduled_t_ear:
        file.write("Trip %i \n" % uid)
        file.write("\t batch: %i\n" % trip["batch"])
        file.write("\t waypoints: %i" % trip["waypoints"][0])
        file.flush()
        for wp_index in range(1, len(trip["waypoints"])):
            # if nodeType[wp]!=5:
            file.write(", %i" % trip["waypoints"][wp_index])
            file.flush()
        file.write(
            "\n\t t_ear: %.1f" % scheduled_t_ear[uid, trip["waypoints"][0]]
        )
        file.flush()
        for wp_index in range(1, len(trip["waypoints"])):
            # if nodeType[wp]!=5:
            wp = trip["waypoints"][wp_index]
            file.write(", %.1f" % scheduled_t_ear[uid, wp])
            file.flush()
        file.write(
            "\n\t t_lat: %.1f" % scheduled_t_lat[uid, trip["waypoints"][0]]
        )
        file.flush()
        for wp_index in range(1, len(trip["waypoints"])):
            # if nodeType[wp]!=5:
            wp = trip["waypoints"][wp_index]
            file.write(", %.1f" % scheduled_t_lat[uid, wp])
            file.flush()
        file.write("\n")
        file.flush()
file.write("slack climb: %f\n" % min(sepClimb_slacks))
file.write("slack cruise: %f\n" % min(sepCruise_slacks))




