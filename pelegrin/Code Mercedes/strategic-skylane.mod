#----------------------------------------
#    Mercedes Pelegrín García			|
#    LIX- École Polytechnique			|
#----------------------------------------

# Strategic deconfliction on skylane corridors by scheduling requests
# Separation constraints: "tan" approach
# Time: departures at integer number of minutes, the rest calculated through motion equations

#-----------Sets: Infraestructure and flights
set A; #aircraft
set AP within A; #aircraft with priority (from previous batches)
set PAIRS := {i in A, j in A : i<>j}; #all pairs (each pair appears twice, in diff order)
set NODES; #nodes of the network
set FATOS within NODES; # take off and landing fatos
set PLANARNODES:=NODES diff FATOS;
set LINKS within{NODES,NODES}; #links of the network (directed)
set LINKSPATH within{A,LINKS}; # Exists the pair (i,l) if link l is in the path of i
set NODESPATH within{A,NODES}; # Exists the pair (i,m) if node m is in the path of i

#-----------Sets: Conflicts
set POT_CONF :={(i,j) in PAIRS, m in NODES: (i,m) in NODESPATH and (j,m) in NODESPATH}; # All potential conflicts (i,j,m)
set POT_CRUISECONF :={(i,j,m) in POT_CONF: m in PLANARNODES}; # All potential cruising conflicts (i,j,m) 

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

#-----------Param: optim. configuration
param D >= 0;
param terr >= 0;
param M:=1000;
param W:=10; #reward for scheduling a priority flight
param epsilon:=0.0000001;
param vcruise;
param vclimb;
param orig{A};
param dist{LINKS}; # distance for each edge 
param tini>= 0; # starting time for scheduling (e.g. 0, 10, 20... min)
param horizon>= 0; # rolling horizon, max scheduled time (default 10 min)
param half_pi = 1.570796; 
param angleJunction{m in PLANARNODES, h in PLANARNODES, l in PLANARNODES: h <> l and
							((m,h) in LINKS or (h,m) in LINKS) and ((m,l) in LINKS or (l,m) in LINKS)};
param S_angle{m in PLANARNODES, h in PLANARNODES, l in PLANARNODES: h <> l and
			((m,h) in LINKS or (h,m) in LINKS) and ((m,l) in LINKS or (l,m) in LINKS)}= 
				if angleJunction[m,h,l]>= half_pi then 1 else 1/sin(angleJunction[m,h,l]);


#---------Variables	
var tmin{NODESPATH} >= 0; #  earlier scheduled arrival time of i to m
var tmax{NODESPATH} >= 0; #  latest scheduled arrival time of i to m
var maxDeparture>= 0;

var tmin_orig{A} integer>= 0; #  tmin_orig[i]=tmin[i,orig[i]]
var tmax_orig{A} integer>= 0; #  tmax_orig[i]=tmax[i,orig[i]]

var scheduled{A} binary; # 1 if trip is scheduled
var passFirst{POT_CONF} binary; # 1 if i passes through m before j
var numScheduled_nonpriority>= 0;
var numScheduled_priority>= 0;

#----------Obj function
maximize ScheduledTrips: numScheduled_nonpriority+W*numScheduled_priority;  #goal 1
minimize MaxDeparture: maxDeparture; # goal 2

#----------Constraints
# 0. -- Objective-related constraints
s.t. goal1Const1: numScheduled_nonpriority= sum{i in A diff AP} scheduled[i]; 
s.t. goal1Const2: numScheduled_priority= sum{i in AP} scheduled[i];
s.t. goal2Const{i in A}: maxDeparture>= tmax[i,orig[i]];

# 1. -- Batch time bounds, integrality at origin, interval width 
s.t. iniTimeBatch{i in A}: tmin_orig[i]>= tini;
s.t. endTimeBatch{i in A}: tmax_orig[i]<= tini+horizon;
s.t. tmin_origin{i in A}: tmin[i,orig[i]]=tmin_orig[i];
s.t. tmax_origin{i in A}: tmax[i,orig[i]]=tmax_orig[i];
s.t. width_origin{i in A}: tmax_orig[i]=tmin_orig[i]+terr;

# 2. -- Arrival times correspond to vcruise and vclimb
s.t. tmin_cruise{(i,m,h) in LINKSPATH: m not in FATOS and h not in FATOS}: tmin[i,h]= tmin[i,m]+dist[m,h]/vcruise;
s.t. tmax_cruise{(i,m,h) in LINKSPATH: m not in FATOS and h not in FATOS}: tmax[i,h]= tmax[i,m]+dist[m,h]/vcruise;
s.t. tmin_climb{(i,m,h) in LINKSPATH: m in FATOS or h in FATOS}: tmin[i,h]= tmin[i,m]+dist[m,h]/vclimb;
s.t. tmax_climb{(i,m,h) in LINKSPATH: m in FATOS or h in FATOS}: tmax[i,h]= tmax[i,m]+dist[m,h]/vclimb;

# 3a. -- Separation at cruise level: crossing conflicts (j traverses m after i)
# --Type A: leg i=1, leg j=1 (merging towards m)
s.t. sepCross_A{(i,j,m,h_i,l_i,h_j,l_j) in CROSS_CONF}: 
		tmin[j,m]-tmax[i,m] >= (D/vcruise)*S_angle[m,h_i,h_j]*passFirst[i,j,m]-M*(1-passFirst[i,j,m]);
s.t. sepMerge{(i,j,m,h_i,h_j,l) in MERGE_CONF}: 
		tmin[j,m]-tmax[i,m] >= (D/vcruise)*S_angle[m,h_i,h_j]*passFirst[i,j,m]-M*(1-passFirst[i,j,m]);
# --Type B: leg i=2, leg j=1 (j approaching m, i leaving m)
s.t. sepCross_B{(i,j,m,h_i,l_i,h_j,l_j) in CROSS_CONF}: 
		tmin[j,m]-tmax[i,m] >= (2*D/vcruise)*S_angle[m,h_j,l_i]*passFirst[i,j,m]-M*(1-passFirst[i,j,m]);
# --Type C: leg i=2, leg j=2 (paths diverging at m)						  
s.t. sepCross_C{(i,j,m,h_i,l_i,h_j,l_j) in CROSS_CONF}: 
		tmin[j,m]-tmax[i,m] >= (D/vcruise)*S_angle[m,l_i,l_j]*passFirst[i,j,m]-M*(1-passFirst[i,j,m]);
s.t. sepSplit{(i,j,m,h,l_i,l_j) in SPLIT_CONF}: 
		tmin[j,m]-tmax[i,m] >= (D/vcruise)*S_angle[m,l_i,l_j]*passFirst[i,j,m]-M*(1-passFirst[i,j,m]);		

# 3b. -- Separation at cruise level: trailing conflicts
s.t. sepTrail_ini{(i,j,m,h) in TRAIL_CRUISE}: (tmin[j,m]-tmax[i,m]) >= (D/vcruise)*passFirst[i,j,m]-M*(1-passFirst[i,j,m]);
s.t. sepTrail_end{(i,j,m,h) in TRAIL_CRUISE}: (tmin[j,h]-tmax[i,h]) >= (D/vcruise)*passFirst[i,j,m]-M*(1-passFirst[i,j,m]);

# 3c. -- Separation at climbing from/to FATOS
s.t. sepTrail_land{(i,j,m,h) in TRAIL_LAND}: (tmin[j,m]-tmax[i,m]) >= passFirst[i,j,m]-M*(1-passFirst[i,j,m]);
s.t. sepTrail_takeoff{(i,j,m,h) in TRAIL_TAKEOFF}: (tmin[j,m]-tmax[i,m]) >= passFirst[i,j,m]-M*(1-passFirst[i,j,m]);
s.t. sepSplit_takeoff{(i,j,m,l_i,l_j) in SPLIT_TAKEOFF}: (tmin[j,m]-tmax[i,m]) >= passFirst[i,j,m]-M*(1-passFirst[i,j,m]);
s.t. sepMerge_land{(i,j,m,h_i,h_j) in MERGE_LAND}: (tmin[j,m]-tmax[i,m]) >= passFirst[i,j,m]-M*(1-passFirst[i,j,m]);

# 4. Binary variables relations
s.t. bin1{(i,j,m) in POT_CONF}: passFirst[i,j,m]+passFirst[j,i,m]<= scheduled[i];
s.t. bin2{(i,j,m) in POT_CONF}: passFirst[i,j,m]+passFirst[j,i,m]<= scheduled[j];
s.t. bin3{(i,j,m) in POT_CONF}: passFirst[i,j,m]+passFirst[j,i,m]>= scheduled[i]+scheduled[j]-1;

# 5. -- No overtaking
s.t. noOverTake{(i,j,m,h) in TRAIL_CONF}: passFirst[i,j,m]<=passFirst[i,j,h];

