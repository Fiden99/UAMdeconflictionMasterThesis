#----------------------------------------
#    Mercedes Pelegrín García			|
#    LIX- École Polytechnique			|
#----------------------------------------

# Tactical deconfliction on skylane corridors by speed regulation
# Separation constraints: "tan" approach

# Discretizad time instants. Time granularity 1 min (instants 0,1,2,3,4,5...) for take off.
# "Exact" early and late arrival times are calculated throughout the junctions. These times are (potentially)
# the result of applying the same speed adjustment.  Then, the extremes of the intervals
# of in-conflict vehicles are separated. 
# As a postprocessing, exact arrival times (e.g. [13.4673,14.4673]) are rounded to next
# greater and smaller numbers with one decimal, to get an interval (e.g. [13.5,14.4]) which is contained in the 
# interval output by the model (i.e. [13.5,14.4] in [13.4673,14.4673]).

# Passing order is kept for every pair (i,j) such that not i nor j are in the DELAYED set
# if i or j are in DELAYED, we will have a binary variable to decide which one traverses a junction first

#-----------Sets: Infraestructure and flights
set A; #aircraft
set GROUND within A; #aircraft on ground when TD is called
set INFLIGHT:=A diff GROUND;
set DELAYED within GROUND;
set AP within A; #aircraft with priority (collaborative intruder)
set AP_NC within A; #aircraft not modifiable (non-collaborative intruder)
set NOT_ORDERED_TRIPS:=DELAYED union AP union AP_NC; # the passing order between these trips and the rest is not decided

set PAIRS := {i in A, j in A : i<>j}; #all pairs (each pair appears twice, in diff order)
set NODES; #nodes of the network
set FATOS within NODES; # take off and landing fatos
set PLANARNODES:=NODES diff FATOS;
set LINKS within{NODES,NODES}; #links of the network (directed)
set LINKSPATH within{A,LINKS}; # Exists the pair (i,l) if link l is in the path of i
set NODESPATH within{A,NODES}; # Exists the pair (i,m) if node m is in the path of i

#----------Param: Schedule
param tplanMin{NODESPATH}; # minimum planned arrival time of i to node m
param tplanMax{NODESPATH}; # maximum planned arrival time of i to node m
param tini>= 0; # actual instant time (no changes are allowed before tini, this is the "past")
param orig{A};
param dest{A};

#-----------Sets: nodes that have already been traversed by trips
set PASSED_NODESPATH within NODESPATH:={(i,m) in NODESPATH:  i in AP_NC or (i in INFLIGHT and tplanMin[i,m]<=tini)};
set PASSED_LINKSPATH within LINKSPATH:={(i,m,h) in LINKSPATH: (i,m) in PASSED_NODESPATH and (i,h) in PASSED_NODESPATH};

set NOTPASSED_NODESPATH:=NODESPATH diff PASSED_NODESPATH;
set NOTPASSED_LINKSPATH:={(i,m,h) in LINKSPATH diff PASSED_LINKSPATH:(i,m) in NOTPASSED_NODESPATH and (i,h) in NOTPASSED_NODESPATH};

# first node to be traversed by i after tini, if i not in ground
set FIRST_NOTPASSED within NOTPASSED_NODESPATH:={(i,m) in NOTPASSED_NODESPATH: i in INFLIGHT and
										not exists{(j,r) in NOTPASSED_NODESPATH} (j==i and tplanMin[j,r]<tplanMin[i,m])};

# inflight nodes for which scheduled time cannot be modified, i.e., speed regulation not possible 										
set FIXED_NODESPATH:=PASSED_NODESPATH union FIRST_NOTPASSED;
set NOTFIXED_NODESPATH:= NOTPASSED_NODESPATH diff FIRST_NOTPASSED;

# waypoints for which we would like to count deviation from SD
set WP_DEV:=(NOTFIXED_NODESPATH diff {(i,m) in NOTFIXED_NODESPATH: i in AP or i in AP_NC}) diff 
			{(i,m) in NOTFIXED_NODESPATH: (i,orig[i],m) in LINKSPATH or m==dest[i]};

#-----------Sets: Conflicts
set POT_CONF :={(i,j) in PAIRS, m in NODES: (i,m) in NODESPATH and (j,m) in NODESPATH
					and (tplanMin[i,m] < tplanMin[j,m] or (tplanMin[i,m]== tplanMin[j,m] and i in AP))}; 
					# All potential conflicts (i,j,m) such that i passes first through m
set POT_CRUISECONF :={(i,j,m) in POT_CONF: m in PLANARNODES}; 
					# All potential cruising conflicts (i,j,m) such that i passes first through m

set TRAIL_CONF :={(i,j,m) in POT_CONF, h in NODES: (i,m,h) in LINKSPATH and (j,m,h) in LINKSPATH}; # Trailing conflicts
set TRAIL_LAND :={(i,j,m,h) in TRAIL_CONF: h in FATOS};
set TRAIL_TAKEOFF :={(i,j,m,h) in TRAIL_CONF: m in FATOS};
set TRAIL_CRUISE:= (TRAIL_CONF diff TRAIL_LAND) diff TRAIL_TAKEOFF;

set CROSS_CONF :={(i,j,m) in POT_CRUISECONF, h_i in PLANARNODES, l_i in PLANARNODES,h_j in PLANARNODES, l_j in PLANARNODES:  
			(i,h_i,m) in LINKSPATH and (i,m,l_i) in LINKSPATH and
			(j,h_j,m) in LINKSPATH and (j,m,l_j) in LINKSPATH and
			 h_i <> h_j and  l_i <> l_j and l_i <> h_j and h_i <> l_j}; # Crossing conflicts
set MERGE_CONF :={(i,j,m) in POT_CRUISECONF, h_i in PLANARNODES, h_j in PLANARNODES, l in PLANARNODES:  
			(i,h_i,m) in LINKSPATH and (i,m,l) in LINKSPATH and
			(j,h_j,m) in LINKSPATH and (j,m,l) in LINKSPATH and
			 h_i <> h_j }; # On merge conflicts
set SPLIT_CONF :={(i,j,m) in POT_CRUISECONF, h in PLANARNODES, l_i in PLANARNODES, l_j in PLANARNODES:  
			(i,h,m) in LINKSPATH and (i,m,l_i) in LINKSPATH and
			(j,h,m) in LINKSPATH and (j,m,l_j) in LINKSPATH and
			 l_i <> l_j }; # Splitting conflicts

set MERGE_LAND:= {(i,j,m) in POT_CONF, h_i in NODES, h_j in NODES: (i,h_i,m) in LINKSPATH and (j,h_j,m) in LINKSPATH 
						and h_i <> h_j and m in FATOS};
set SPLIT_TAKEOFF :={(i,j,m) in POT_CONF, l_i in NODES, l_j in NODES: (i,m,l_i) in LINKSPATH and (j,m,l_j) in LINKSPATH 
						and l_i <> l_j and m in FATOS};

set TRAIL_CONF_1:={(i,j,m,h) in TRAIL_CONF: i not in NOT_ORDERED_TRIPS and j not in NOT_ORDERED_TRIPS};
set TRAIL_LAND_1:={(i,j,m,h) in TRAIL_LAND: i not in NOT_ORDERED_TRIPS and j not in NOT_ORDERED_TRIPS};
set TRAIL_TAKEOFF_1:={(i,j,m,h) in TRAIL_TAKEOFF: i not in NOT_ORDERED_TRIPS and j not in NOT_ORDERED_TRIPS};
set TRAIL_CRUISE_1:={(i,j,m,h) in TRAIL_CRUISE: i not in NOT_ORDERED_TRIPS and j not in NOT_ORDERED_TRIPS};
set CROSS_CONF_1:={(i,j,m,h_i,l_i,h_j,l_j) in CROSS_CONF: i not in NOT_ORDERED_TRIPS and j not in NOT_ORDERED_TRIPS};
set MERGE_CONF_1:={(i,j,m,h_i,h_j,l) in MERGE_CONF: i not in NOT_ORDERED_TRIPS and j not in NOT_ORDERED_TRIPS};
set SPLIT_CONF_1:={(i,j,m,h,l_i,l_j) in SPLIT_CONF: i not in NOT_ORDERED_TRIPS and j not in NOT_ORDERED_TRIPS};
set MERGE_LAND_1:={(i,j,m,h_i,h_j) in MERGE_LAND: i not in NOT_ORDERED_TRIPS and j not in NOT_ORDERED_TRIPS};
set SPLIT_TAKEOFF_1:={(i,j,m,l_i,l_j) in SPLIT_TAKEOFF: i not in NOT_ORDERED_TRIPS and j not in NOT_ORDERED_TRIPS};
set TRAIL_LAND_2:=TRAIL_LAND diff TRAIL_LAND_1;
set TRAIL_TAKEOFF_2:=TRAIL_TAKEOFF diff TRAIL_TAKEOFF_1;
set TRAIL_CRUISE_2:=TRAIL_CRUISE diff TRAIL_CRUISE_1;
set CROSS_CONF_2:=CROSS_CONF diff CROSS_CONF_1;
set MERGE_CONF_2:=MERGE_CONF diff MERGE_CONF_1;
set SPLIT_CONF_2:=SPLIT_CONF diff SPLIT_CONF_1;
set MERGE_LAND_2:=MERGE_LAND diff MERGE_LAND_1;
set SPLIT_TAKEOFF_2:=SPLIT_TAKEOFF diff SPLIT_TAKEOFF_1;

#-----------Param: optim. configuration
param D >= 0;
param M:=1000;
param W:=10000; #penaliz for 1 minute of delay departure of priority traffic
param vmin;
param vmax;
param vclimb;
param dist{LINKS}; # distance for each edge 
param half_pi = 1.570796; 
param angleJunction{m in PLANARNODES, h in PLANARNODES, l in PLANARNODES: h <> l and
							((m,h) in LINKS or (h,m) in LINKS) and ((m,l) in LINKS or (l,m) in LINKS)};
param S_angle{m in PLANARNODES, h in PLANARNODES, l in PLANARNODES: h <> l and
			((m,h) in LINKS or (h,m) in LINKS) and ((m,l) in LINKS or (l,m) in LINKS)}= 
				if angleJunction[m,h,l]>= half_pi then 1 else 1/sin(angleJunction[m,h,l]);


#---------Variables	
#  re-scheduled arrival time of i to m: [tmin,tmax]
var tmin{NODESPATH}; #  earlier new arrival time of i to m
var tmax{NODESPATH}; #  latest new arrival time of i to m
var tmin_orig{A} integer>= 0; #  tmin[i,orig[i]]=tmin_orig[i]

# bounds on arrival times:
var t_lb{NOTFIXED_NODESPATH}; # lower bound on tmin, corresponds with vmin
var t_ub{NOTFIXED_NODESPATH}; # upper bound on tmin, corresponds with vmax

# deviation from plan: dev[i,m]= abs(tmin[i,m]-tplanMin[i,m])
var dev{WP_DEV} >= 0; 
var delayPriority{AP} integer >= 0; # delay in departure of priority traffic 

# passing order
var passFirst{(i,j,m) in POT_CONF: i in NOT_ORDERED_TRIPS or j in NOT_ORDERED_TRIPS} binary; # 1 if i passes through m before j

#----------Obj function
minimize Dev:   sum{i in AP} W*delayPriority[i] # 1st goal
			  + sum{(i,m) in WP_DEV} dev[i,m];  # 2nd goal

#----------Constraints
# TODO: should this constraints be removed???
# constraints to restrict dev to a value close to 0 (should be dropped)
#s.t. fixDev{(i,m) in WP_DEV}: dev[i,m] <= 0.09999999;

# 1. Objective 
# goal 2 = abs deviation (these constraints work because of the minimizing objective)
s.t. abs1{(i,m) in WP_DEV}: dev[i,m] >= tmin[i,m]-tplanMin[i,m];
s.t. abs2{(i,m) in WP_DEV}: dev[i,m] >= -tmin[i,m]+tplanMin[i,m];

# 2. Priority traffic implement planned trajectory or delayPriority[i]!=0
s.t. priority1{(i,m) in NODESPATH: i in AP}: tmin[i,m] =tplanMin[i,m]+delayPriority[i];
s.t. priority2{(i,m) in NODESPATH: i in AP}: tmax[i,m] =tplanMax[i,m]+delayPriority[i];

# 3. Modelling of past and future
# Ground traffic cannot depart before tini
s.t. ground_departure{i in GROUND}: tmin_orig[i]>=tini;
# What happened in the past stay fixed
s.t. tmin_past{(i,m) in FIXED_NODESPATH}: tmin[i,m]=tplanMin[i,m];
s.t. tmax_past{(i,m) in FIXED_NODESPATH}: tmax[i,m]=tplanMax[i,m];

# 4. Recommended travel times define intervals [tmin,tmax] and keep scheduled passing order
s.t. timeMargin{(i,m) in NOTFIXED_NODESPATH: i not in AP}:  tmax[i,m]=tmin[i,m]+tplanMax[i,m]-tplanMin[i,m];
s.t. passOrder_trail{(i,j,m,h) in TRAIL_CONF_1}: tmax[i,m] <= tmin[j,m];
s.t. passOrder_cross{(i,j,m,h_i,l_i,h_j,l_j) in CROSS_CONF_1}: tmax[i,m] <= tmin[j,m];
s.t. passOrder_merge{(i,j,m,h_i,h_j,l) in MERGE_CONF_1}: tmax[i,m]<= tmin[j,m];
s.t. passOrder_split{(i,j,m,h_i,h_j,l) in SPLIT_CONF_1}: tmax[i,m] <= tmin[j,m];

# 5. -- Integrality at origin 
s.t. tmin_origin{i in GROUND}: tmin[i,orig[i]]=tmin_orig[i];

# 6. Speed is kept in [vmin,vmax] 
s.t. LBv{(i,m,h) in NOTPASSED_LINKSPATH: i not in AP and m not in FATOS and h not in FATOS}: t_lb[i,h]=  tmin[i,m]+dist[m,h]/vmax;
s.t. UBv{(i,m,h) in NOTPASSED_LINKSPATH: i not in AP and m not in FATOS and h not in FATOS}: t_ub[i,h]=  tmin[i,m]+dist[m,h]/vmin;
s.t. LBv_climb{(i,m,h) in NOTPASSED_LINKSPATH: i not in AP and (m in FATOS or h in FATOS)}: t_lb[i,h]=  tmin[i,m]+dist[m,h]/vclimb;
s.t. UBv_climb{(i,m,h) in NOTPASSED_LINKSPATH: i not in AP and (m in FATOS or h in FATOS)}: t_ub[i,h]=  tmin[i,m]+dist[m,h]/vclimb;
#s.t. LBv_climb{(i,m,h) in NOTPASSED_LINKSPATH: i not in AP and (m in FATOS or h in FATOS)}: t_lb[i,h]=  tmin[i,m]+ceil((dist[m,h]/vclimb)*10)/10;
#s.t. UBv_climb{(i,m,h) in NOTPASSED_LINKSPATH: i not in AP and (m in FATOS or h in FATOS)}: t_ub[i,h]=  tmin[i,m]+ceil((dist[m,h]/vclimb)*10)/10;
#s.t. LBv_takeoff{(i,m,h) in NOTPASSED_LINKSPATH: i not in AP and m in FATOS}: t_lb[i,h]=  tmin[i,m]+ceil((dist[m,h]/vclimb)*10)/10;
#s.t. UBv_takeoff{(i,m,h) in NOTPASSED_LINKSPATH: i not in AP and m in FATOS}: t_ub[i,h]=  tmin[i,m]+ceil((dist[m,h]/vclimb)*10)/10;
#s.t. LBv_landing{(i,m,h) in NOTPASSED_LINKSPATH: i not in AP and h in FATOS}: t_lb[i,h]=  tmin[i,m]+dist[m,h]/vclimb;
#s.t. UBv_landing{(i,m,h) in NOTPASSED_LINKSPATH: i not in AP and h in FATOS}: t_ub[i,h]=  tmin[i,m]+dist[m,h]/vclimb;
s.t. LBt{(i,m) in NOTFIXED_NODESPATH: i not in AP}: tmin[i,m] >= t_lb[i,m];
s.t. UBt{(i,m) in NOTFIXED_NODESPATH: i not in AP}: tmin[i,m] <= t_ub[i,m];

##-------- SEPARATION CONSTRAINTS. a) Passage order is known:
# 5. Separation at cruise level: crossing conflicts (j traverses m after i)
# --Type A: leg i=1, leg j=1 (merging towards m)
s.t. sepCross_A{(i,j,m,h_i,l_i,h_j,l_j) in CROSS_CONF_1}: 
		tmin[j,m]-tmax[i,m] >= (D/vmin)*S_angle[m,h_i,h_j]; #tmin[j,m]-tmax[i,m] >= (D/vmin[j,m])*S_angle[m,h_i,h_j];
s.t. sepMerge{(i,j,m,h_i,h_j,l) in MERGE_CONF_1}: 
		tmin[j,m]-tmax[i,m] >= (D/vmin)*S_angle[m,h_i,h_j]; #tmin[j,m]-tmax[i,m] >= (D/vmin[j,m])*S_angle[m,h_i,h_j];
# --Type B: leg i=2, leg j=1 (j approaching m, i leaving m)
s.t. sepCross_B{(i,j,m,h_i,l_i,h_j,l_j) in CROSS_CONF_1}: 
		tmin[j,m]-tmax[i,m] >= (2*D/vmin) * S_angle[m,h_j,l_i]; #tmin[j,m]-tmax[i,m] >= (D/vmin[i,l_i]+D/vmin[j,m]) * S_angle[m,h_j,l_i];
# --Type C: leg i=2, leg j=2 (paths diverging at m)						  
s.t. sepCross_C{(i,j,m,h_i,l_i,h_j,l_j) in CROSS_CONF_1}: 
		tmin[j,m]-tmax[i,m] >= (D/vmin) * S_angle[m,l_i,l_j]; #tmin[j,m]-tmax[i,m] >= (D/vmin[i,l_i]) * S_angle[m,l_i,l_j];
s.t. sepSplit{(i,j,m,h,l_i,l_j) in SPLIT_CONF_1}: 
		tmin[j,m]-tmax[i,m] >= (D/vmin) * S_angle[m,l_i,l_j]; #tmin[j,m]-tmax[i,m] >= (D/vmin[i,l_i]) * S_angle[m,l_i,l_j];		

# 6. Separation at cruise level: trailing conflicts
s.t. sepTrail_ini{(i,j,m,h) in TRAIL_CRUISE_1}: (tmin[j,m]-tmax[i,m]) >= D/vmin; #(tmin[j,m]-tmax[i,m]) >= D/vmin[i,h];
s.t. sepTrail_end{(i,j,m,h) in TRAIL_CRUISE_1}: (tmin[j,h]-tmax[i,h]) >= D/vmin; #(tmin[j,h]-tmax[i,h]) >= D/vmin[j,h];

# 7. Separation at climbing from/to FATOS
s.t. sepTrail_land{(i,j,m,h) in TRAIL_LAND_1}: (tmin[j,m]-tmax[i,m]) >= 1;
s.t. sepTrail_takeoff{(i,j,m,h) in TRAIL_TAKEOFF_1}: (tmin[j,m]-tmax[i,m]) >= 1;
s.t. sepSplit_takeoff{(i,j,m,l_i,l_j) in SPLIT_TAKEOFF_1}: (tmin[j,m]-tmax[i,m]) >= 1;
s.t. sepMerge_land{(i,j,m,h_i,h_j) in MERGE_LAND_1}: (tmin[j,m]-tmax[i,m]) >= 1;

##-------- SEPARATION CONSTRAINTS. b) Passage order is to be decided:
# 5. Separation at cruise level: crossing conflicts (j traverses m after i)
# --Type A: leg i=1, leg j=1 (merging towards m)
s.t. sepCross_A_i{(i,j,m,h_i,l_i,h_j,l_j) in CROSS_CONF_2}: 
		tmin[j,m]-tmax[i,m] >= (D/vmin)*S_angle[m,h_i,h_j]*passFirst[i,j,m]-M*(1-passFirst[i,j,m]);
s.t. sepMerge_i{(i,j,m,h_i,h_j,l) in MERGE_CONF_2}: 
		tmin[j,m]-tmax[i,m] >= (D/vmin)*S_angle[m,h_i,h_j]*passFirst[i,j,m]-M*(1-passFirst[i,j,m]); 
# --Type B: leg i=2, leg j=1 (j approaching m, i leaving m)
s.t. sepCross_B_i{(i,j,m,h_i,l_i,h_j,l_j) in CROSS_CONF_2}: 
		tmin[j,m]-tmax[i,m] >= (2*D/vmin) * S_angle[m,h_j,l_i]*passFirst[i,j,m]-M*(1-passFirst[i,j,m]);
# --Type C: leg i=2, leg j=2 (paths diverging at m)						  
s.t. sepCross_C_i{(i,j,m,h_i,l_i,h_j,l_j) in CROSS_CONF_2}: 
		tmin[j,m]-tmax[i,m] >= (D/vmin) * S_angle[m,l_i,l_j]*passFirst[i,j,m]-M*(1-passFirst[i,j,m]); 
s.t. sepSplit_i{(i,j,m,h,l_i,l_j) in SPLIT_CONF_2}: 
		tmin[j,m]-tmax[i,m] >= (D/vmin) * S_angle[m,l_i,l_j]*passFirst[i,j,m]-M*(1-passFirst[i,j,m]); 

# --Type A: leg i=1, leg j=1 (merging towards m)
s.t. sepCross_A_j{(i,j,m,h_i,l_i,h_j,l_j) in CROSS_CONF_2}: 
		tmin[i,m]-tmax[j,m] >= (D/vmin)*S_angle[m,h_i,h_j]*(1-passFirst[i,j,m])-M*passFirst[i,j,m];
s.t. sepMerge_j{(i,j,m,h_i,h_j,l) in MERGE_CONF_2}: 
		tmin[i,m]-tmax[j,m] >= (D/vmin)*S_angle[m,h_i,h_j]*(1-passFirst[i,j,m])-M*passFirst[i,j,m];
# --Type B: leg i=2, leg j=1 (j approaching m, i leaving m)
s.t. sepCross_B_j{(i,j,m,h_i,l_i,h_j,l_j) in CROSS_CONF_2}: 
		tmin[i,m]-tmax[j,m] >= (2*D/vmin) * S_angle[m,h_j,l_i]*(1-passFirst[i,j,m])-M*passFirst[i,j,m];
# --Type C: leg i=2, leg j=2 (paths diverging at m)						  
s.t. sepCross_C_j{(i,j,m,h_i,l_i,h_j,l_j) in CROSS_CONF_2}: 
		tmin[i,m]-tmax[j,m] >= (D/vmin) * S_angle[m,l_i,l_j]*(1-passFirst[i,j,m])-M*passFirst[i,j,m];
s.t. sepSplit_j{(i,j,m,h,l_i,l_j) in SPLIT_CONF_2}: 
		tmin[i,m]-tmax[j,m] >= (D/vmin) * S_angle[m,l_i,l_j]*(1-passFirst[i,j,m])-M*passFirst[i,j,m];

# 6. Separation at cruise level: trailing conflicts
s.t. sepTrail_ini_i{(i,j,m,h) in TRAIL_CRUISE_2}: (tmin[j,m]-tmax[i,m]) >= (D/vmin)*passFirst[i,j,m]-M*(1-passFirst[i,j,m]); 
s.t. sepTrail_end_i{(i,j,m,h) in TRAIL_CRUISE_2}: (tmin[j,h]-tmax[i,h]) >= (D/vmin)*passFirst[i,j,m]-M*(1-passFirst[i,j,m]); 

s.t. sepTrail_ini_j{(i,j,m,h) in TRAIL_CRUISE_2}: (tmin[i,m]-tmax[j,m]) >= (D/vmin)*(1-passFirst[i,j,m])-M*passFirst[i,j,m]; 
s.t. sepTrail_end_j{(i,j,m,h) in TRAIL_CRUISE_2}: (tmin[i,h]-tmax[j,h]) >= (D/vmin)*(1-passFirst[i,j,m])-M*passFirst[i,j,m];  

# 7. Separation at climbing from/to FATOS
s.t. sepTrail_land_i{(i,j,m,h) in TRAIL_LAND_2}: tmin[j,m]-tmax[i,m] >= passFirst[i,j,m]-M*(1-passFirst[i,j,m]);
s.t. sepTrail_takeoff_i{(i,j,m,h) in TRAIL_TAKEOFF_2}: tmin[j,m]-tmax[i,m] >= passFirst[i,j,m]-M*(1-passFirst[i,j,m]);
s.t. sepSplit_takeoff_i{(i,j,m,l_i,l_j) in SPLIT_TAKEOFF_2}: tmin[j,m]-tmax[i,m] >= passFirst[i,j,m]-M*(1-passFirst[i,j,m]);
s.t. sepMerge_land_i{(i,j,m,h_i,h_j) in MERGE_LAND_2}: tmin[j,m]-tmax[i,m] >= passFirst[i,j,m]-M*(1-passFirst[i,j,m]);

s.t. sepTrail_land_j{(i,j,m,h) in TRAIL_LAND_2}: tmin[i,m]-tmax[j,m] >= 1-M*passFirst[i,j,m];
s.t. sepTrail_takeoff_j{(i,j,m,h) in TRAIL_TAKEOFF_2}: tmin[i,m]-tmax[j,m] >= 1-M*passFirst[i,j,m];
s.t. sepSplit_takeoff_j{(i,j,m,l_i,l_j) in SPLIT_TAKEOFF_2}: tmin[i,m]-tmax[j,m] >= 1-M*passFirst[i,j,m];
s.t. sepMerge_land_j{(i,j,m,h_i,h_j) in MERGE_LAND_2}: tmin[i,m]-tmax[j,m] >= 1-M*passFirst[i,j,m];

