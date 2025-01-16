#data
param nn; #number of nodes
set F; #set of flights
set V:= 0..nn-1; #set of nodes
set E within {V cross V}; #arcs
param s{F}; #starting point for each flight
param e{F}; #ending poitn for each flight
param d{E}>=0; #distance for each pair of nodes
param v_max;
param v_min; #param v{F,V}; #entering speed
param bigM:=1000; #bigM for linearizing purpose
set conflictsNodes within {x in V, x1 in V, x2 in V: x1<>x2 and ((x,x1) in E or (x1,x) in E) and ((x,x2) in E or (x2,x) in E)};
param angle{conflictsNodes} ; #angle -+ divering
param D; # safety distance
param t_hat_ear{F,V} default 997;
param t_hat_lat{F,V} default 999;
param drifted_flight default -1;
param drifted_wp default -1;
param drifted_t_ear_fix  default -1;
param drifted_t_lat_fix  default -1;
set fixedFlights within {F cross E};

#definition of sets and parameters not in .dat
set fixedF = setof {(f,x,y) in fixedFlights} f;
set freeF = F diff fixedF;
set fixedFNodes = {setof {(f,x,y) in fixedFlights} (f,x)} union {setof {(f,x,y) in fixedFlights} (f,y)};
param wFixed{(x,y) in E,f in fixedF} binary = if (f,x,y) in fixedFlights then 1 else 0;

#variables
var w{E,freeF} binary; #flight f pass through arc i,j
var t_down{freeF,V} >=0;
var t_up {freeF,V} >= 0;
var t_ear{freeF,V} >=0; 
var t_lat{freeF,V} >=0; 

var t_down_fixed{fixedFNodes} >=0;
var t_up_fixed {fixedFNodes} >= 0;
var t_ear_fixed{fixedFNodes} >=0; 
var t_lat_fixed{fixedFNodes} >=0; 

var t_ear_start{F} integer >=0;
var abs_t_ear{F} >=0;

#binary variables for linearization of conflicts
var passFirst{i in freeF, j in freeF, x in V: i<>j} binary;
var passFixed{i in fixedF, j in fixedF, x in V: i<>j and (i,x) in fixedFNodes and (j,x) in fixedFNodes} binary;
var passI{i in fixedF, j in freeF, x in V: i<>j and (i,x) in fixedFNodes} binary;
var passJ{i in freeF, j in fixedF, x in V: i<>j and (j,x) in fixedFNodes} binary;
#constrains
subject to startInteger{f in freeF}:
t_ear[f,s[f]] = t_ear_start[f];

subject to startIntegerFixed{f in fixedF}:
t_ear_fixed[f,s[f]] = t_ear_start[f];

subject to afterprecalculated{f in F}:
t_hat_ear[f,s[f]] <= t_ear_start[f];

subject to calculateLat{i in freeF, x in V}:
t_lat[i,x]=t_ear[i,x] + t_hat_lat[i,x] - t_hat_ear[i,x];

subject to calculateLatFixed{(i,x) in fixedFNodes}:
t_lat_fixed[i,x]=t_ear_fixed[i,x] + t_hat_lat[i,x] - t_hat_ear[i,x];


subject to startingW{i in freeF}:
sum{(x,s[i]) in E} w[x,s[i],i] - sum{(s[i],x) in E} w[s[i],x,i]=-1;
subject to finishingW{i in freeF}:
sum{(x,e[i]) in E} w[x,e[i],i] - sum{(e[i],x) in E} w[e[i],x,i]=1;
subject to allW{i in freeF,x in V : x<> s[i] and x <> e[i]}:
sum{(x,y) in E} w[x,y,i]=sum{(y,x) in E} w[y,x,i];

subject to limitT_down{i in freeF, x in V : x <> s[i]}:
t_down[i,x] <= t_ear[i,x];
subject to limitT_up{i in freeF, x in V: x <> s[i]}:
t_ear[i,x] <= t_up[i,x];

subject to limitT_down_fixed{(i,x) in fixedFNodes : x <> s[i]}:
t_down_fixed[i,x] <= t_ear_fixed[i,x];
subject to limitT_up_fixed{(i,x) in fixedFNodes: x <> s[i]}:
t_ear_fixed[i,x] <= t_up_fixed[i,x];


subject to defineT_down{f in freeF,y in V: y <> s[f]}:
t_down[f,y]=sum{(x,y) in E} w[x,y,f] *(d[x,y]/v_max + t_ear[f,x]);
subject to defineT_up{f in freeF,y in V: y <> s[f]}:
t_up[f,y]=sum{(x,y) in E} w[x,y,f] * (d[x,y]/v_min + t_ear[f,x]);

subject to defineT_down_fixed{(f,x,y) in fixedFlights}:
t_down_fixed[f,y]=d[x,y]/v_max + t_ear_fixed[f,x];
subject to defineT_up_fixed{(f,x,y) in fixedFlights}:
t_up_fixed[f,y]=d[x,y]/v_min + t_ear_fixed[f,x];

# conflicts

subject to trail13 {i in freeF,j in freeF, (x,y) in E: i<>j and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x,y,i] + w[x,y,j] == 2) ==> (t_ear[j,x]-t_lat[i,x] >= D/v_min* passFirst[i,j,x] -bigM* (1-passFirst[i,j,x]));
subject to trail14 {i in freeF,j in freeF, (x,y) in E: i<>j and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x,y,i] + w[x,y,j] == 2) ==> (t_ear[i,x]-t_lat[j,x]>=D/v_min* (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);

subject to trail13Fixed {i in fixedF,j in fixedF, (x,y) in E: i<>j and (wFixed[x,y,i] + wFixed[x,y,j] == 2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[j,x]-t_lat_fixed[i,x] >= D/v_min-bigM* passFixed[i,j,x] - bigM* (1-passFixed[i,j,x]);
subject to trail14Fixed {i in fixedF,j in fixedF, (x,y) in E: i<>j and (wFixed[x,y,i] + wFixed[x,y,j] == 2) and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[i,x]-t_lat_fixed[j,x]>=D/v_min* (1-passFixed[i,j,x]) - bigM*passFixed[i,j,x];

subject to trail13FixedI {i in fixedF,j in freeF, (x,y) in E: (wFixed[x,y,i]==1) and i<>j and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[j,x]-t_lat_fixed[i,x] >= D/v_min* passI[i,j,x] - bigM*(1-passI[i,j,x]) - bigM*(1-w[x,y,j]);
subject to trail14FixedI {i in fixedF,j in freeF, (x,y) in E: (wFixed[x,y,i]==1) and i<>j and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[i,x]-t_lat[j,x]>=D/v_min* (1-passI[i,j,x]) - bigM*passI[i,j,x] - bigM*(1-w[x,y,j]);

subject to trail13FixedJ {i in freeF,j in fixedF, (x,y) in E: (wFixed[x,y,j]==1) and i<>j and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[j,x]-t_lat[i,x] >= D/v_min* passJ[i,j,x] -bigM* (1-passJ[i,j,x]) - bigM*(1-w[x,y,i]);
subject to trail14FixedJ {i in freeF,j in fixedF, (x,y) in E: (wFixed[x,y,j]==1) and i<>j and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[i,x]-t_lat_fixed[j,x]>=D/v_min* (1-passJ[i,j,x]) - bigM*passJ[i,j,x] - bigM*(1-w[x,y,i]);


subject to trail23 {i in freeF,j in freeF, (x,y) in E: i<>j and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
(w[x,y,i] + w[x,y,j] == 2) ==> t_ear[j,y]-t_lat[i,y]>= D/v_min* passFirst[i,j,x] - bigM*(1-passFirst[i,j,x]);
subject to trail24 {i in freeF,j in freeF, (x,y) in E: i<>j and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
(w[x,y,i] + w[x,y,j] == 2) ==> t_ear[i,y]-t_lat[j,y]>= D/v_min * (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x];

subject to trail23Fixed {i in fixedF,j in fixedF, (x,y) in E: i<>j and(wFixed[x,y,i] + wFixed[x,y,j] == 2) and  (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
t_ear_fixed[j,y]-t_lat_fixed[i,y]>= D/v_min* passFixed[i,j,x] - bigM*(1-passFixed[i,j,x]);
subject to trail24Fixed {i in fixedF,j in fixedF, (x,y) in E: i<>j and (wFixed[x,y,i] + wFixed[x,y,j] == 2) and(t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
t_ear_fixed[i,y]-t_lat_fixed[j,y]>= D/v_min * (1-passFixed[i,j,x]) - bigM*passFixed[i,j,x];

subject to trail23FixedI {i in fixedF,j in freeF, (x,y) in E: i<>j and (wFixed[x,y,i]==1) and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
t_ear[j,y]-t_lat_fixed[i,y]>= D/v_min* passI[i,j,x] - bigM*(1-passI[i,j,x]) - bigM*(1-w[x,y,j]);
subject to trail24FixedI {i in fixedF,j in freeF, (x,y) in E: i<>j and (wFixed[x,y,i]==1) and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
t_ear_fixed[i,y]-t_lat[j,y]>= D/v_min * (1-passI[i,j,x]) - bigM*passI[i,j,x] - bigM*(1-w[x,y,j]);

subject to trail23FixedJ {i in freeF,j in fixedF, (x,y) in E: i<>j and (wFixed[x,y,j]==1) and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
t_ear_fixed[j,y]-t_lat[i,y]>= D/v_min* passJ[i,j,x] - bigM*(1-passJ[i,j,x]) - bigM*(1-w[x,y,i]);
subject to trail24FixedJ {i in freeF,j in fixedF, (x,y) in E: i<>j and (wFixed[x,y,j]==1) and (t_hat_ear[i,y] < t_hat_ear[j,y] or (t_hat_ear[i,y] == t_hat_ear[j,y] and i<j))}:
t_ear[i,y]-t_lat_fixed[j,y]>= D/v_min * (1-passJ[i,j,x]) - bigM*passJ[i,j,x] - bigM*(1-w[x,y,i]);


#TODO check if the speed parameter is correct for merge4
subject to merge3 {i in freeF, j in freeF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x1,x,i]+w[x2,x,j] == 2) ==> (t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min * passFirst[i,j,x] - bigM*(1-passFirst[i,j,x]));
subject to merge3_1 {i in freeF, j in freeF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x1,x,j]+w[x2,x,i] == 2) ==> (t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min * passFirst[i,j,x] - bigM*(1-passFirst[i,j,x]));

subject to merge3Fixed {i in fixedF, j in fixedF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and (wFixed[x1,x,i]+wFixed[x2,x,j] == 2) and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*D/v_min * passFixed[i,j,x] - bigM*(1-passFixed[i,j,x]);
subject to merge3_1Fixed {i in fixedF, j in fixedF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x1,x,j]+wFixed[x2,x,i] == 2) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*D/v_min * passFixed[i,j,x] - bigM*(1-passFixed[i,j,x]);

subject to merge3FixedI {i in fixedF, j in freeF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x1,x,i]==1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*D/v_min * passI[i,j,x] - bigM*(1-passI[i,j,x]) - bigM*(1-w[x2,x,j]);
subject to merge3_1FixedI {i in fixedF, j in freeF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x2,x,i]==1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*D/v_min * passI[i,j,x] - bigM*(1-passI[i,j,x]) - bigM*(1-w[x1,x,j]);

subject to merge3FixedJ {i in freeF, j in fixedF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x2,x,j] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min * passJ[i,j,x] - bigM*(1-passJ[i,j,x]) - bigM*(1-w[x1,x,i]);
subject to merge3_1FixedJ {i in freeF, j in fixedF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x1,x,j] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min * passJ[i,j,x] - bigM*(1-passJ[i,j,x]) - bigM*(1-w[x2,x,i]);



subject to merge4 {i in freeF, j in freeF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x1,x,j]+w[x2,x,i] == 2) ==> (t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min* (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);
subject to merge4_1 {i in freeF, j in freeF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x1,x,i]+w[x2,x,j] == 2) ==> (t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min* (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);

subject to merge4Fixed {i in fixedF, j in fixedF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x1,x,j]+wFixed[x2,x,i] == 2) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*D/v_min* (1-passFixed[i,j,x]) - bigM*passFixed[i,j,x];
subject to merge4_1Fixed {i in fixedF, j in fixedF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x1,x,i]+wFixed[x2,x,j] == 2) and i<>j  and  x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*D/v_min* (1-passFixed[i,j,x]) - bigM*passFixed[i,j,x];

subject to merge4FixedI {i in fixedF, j in freeF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x2,x,i]==1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min* (1-passI[i,j,x]) - bigM*passI[i,j,x] - bigM*(1-w[x1,x,j]);
subject to merge4_1FixedI {i in fixedF, j in freeF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x1,x,i]==1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min* (1-passI[i,j,x]) - bigM*passI[i,j,x] - bigM*(1-w[x2,x,j]);

subject to merge4FixedJ {i in freeF, j in fixedF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x1,x,j] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*D/v_min* (1-passJ[i,j,x]) - bigM*passJ[i,j,x] - bigM*(1-w[x2,x,i]);
subject to merge4_1FixedJ {i in freeF, j in fixedF, x in V,(x1,x) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x2,x,j]==1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*D/v_min* (1-passJ[i,j,x]) - bigM*passJ[i,j,x] -bigM*(1-w[x1,x,i]);


subject to diver3 {i in freeF, j in freeF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
(w[x,x1,i]+w[x2,x,j] == 2) ==> (t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)*passFirst[i,j,x] - bigM*(1-passFirst[i,j,x])) ;
subject to diver3_1 {i in freeF, j in freeF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
(w[x,x1,j]+w[x2,x,i] == 2) ==> (t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)*passFirst[i,j,x] - bigM*(1-passFirst[i,j,x])) ;

subject to diver3Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,i]+wFixed[x2,x,j] == 2) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
t_ear_fixed[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)*passFixed[i,j,x] - bigM*(1-passFixed[i,j,x]);
subject to diver3_1Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and(wFixed[x,x1,j]+wFixed[x2,x,i] == 2) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
t_ear_fixed[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)*passFixed[i,j,x] - bigM*(1-passFixed[i,j,x]) ;

subject to diver3FixedI {i in fixedF, j in freeF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,i]==1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
t_ear[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)*passI[i,j,x] - bigM*(1-passI[i,j,x]) - bigM*(1-w[x2,x,j]) ;
subject to diver3_1FixedI {i in fixedF, j in freeF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x2,x,i]==1) and  i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
t_ear[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)*passI[i,j,x] - bigM*(1-passI[i,j,x]) - bigM*(1-w[x,x1,j]) ;

subject to diver3FixedJ {i in freeF, j in fixedF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x2,x,j]==1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
t_ear_fixed[j,x]- t_lat[i,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)*passJ[i,j,x] - bigM*(1-passJ[i,j,x]) - bigM*(1-w[x,x1,i]) ;
subject to diver3_1FixedJ {i in freeF, j in fixedF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,j]==1)and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <>e[i]
t_ear_fixed[j,x]- t_lat[i,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)*passJ[i,j,x] - bigM*(1-passJ[i,j,x]) - bigM*(1-w[x2,x,i]) ;



subject to diver4 {i in freeF, j in freeF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes  and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
(w[x,x1,i]+w[x2,x,j] == 2) ==> (t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)* (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);
subject to diver4_1 {i in freeF, j in freeF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
(w[x,x1,j]+w[x2,x,i] == 2) ==> (t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)* (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);

subject to diver4Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and (wFixed[x,x1,i]+wFixed[x2,x,j] == 2) and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
t_ear_fixed[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)* (1-passFixed[i,j,x]) - bigM*passFixed[i,j,x];
subject to diver4_1Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and i<>j and (wFixed[x,x1,j]+wFixed[x2,x,i] == 2) and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
t_ear_fixed[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)* (1-passFixed[i,j,x]) - bigM*passFixed[i,j,x];

subject to diver4FixedI {i in fixedF, j in freeF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,i]==1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
t_ear_fixed[i,x]- t_lat[j,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)* (1-passI[i,j,x]) - bigM*passI[i,j,x] - bigM*(1-w[x2,x,j]);
subject to diver4_1FixedI {i in fixedF, j in freeF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x2,x,i]==1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
t_ear_fixed[i,x]- t_lat[j,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)* (1-passI[i,j,x]) - bigM*passI[i,j,x] - bigM*(1-w[x,x1,j]);

subject to diver4FixedJ {i in freeF, j in fixedF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x2,x,j] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
t_ear[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)* (1-passJ[i,j,x]) - bigM*passJ[i,j,x] - bigM*(1-w[x,x1,i]);
subject to diver4_1FixedJ {i in freeF, j in fixedF, x in V,(x,x1) in E, (x2,x) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,j] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}: #and x <> e[j]
t_ear[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*(D/v_min+D/v_min)* (1-passJ[i,j,x]) - bigM*passJ[i,j,x] - bigM*(1-w[x2,x,i]);



subject to split3 {i in freeF, j in freeF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x,x1,i]+w[x,x2,j] == 2) ==> (t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min*passFirst[i,j,x] - bigM*(1-passFirst[i,j,x]));
subject to split3_1 {i in freeF, j in freeF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x,x1,j]+w[x,x2,i] == 2) ==> (t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min*passFirst[i,j,x] - bigM*(1-passFirst[i,j,x]));

subject to split3Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,i]+wFixed[x,x2,j] == 2) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*D/v_min*passFixed[i,j,x] - bigM*(1-passFixed[i,j,x]);
subject to split3_1Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,j]+wFixed[x,x2,i] == 2) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*D/v_min*passFixed[i,j,x] - bigM*(1-passFixed[i,j,x]);

subject to split3FixedI {i in fixedF, j in freeF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,i] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*D/v_min*passI[i,j,x] - bigM*(1-passI[i,j,x]) - bigM*(1-w[x,x2,j]);
subject to split3_1FixedI {i in fixedF, j in freeF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x2,i] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[j,x]- t_lat_fixed[i,x]>=angle[x,x1,x2]*D/v_min*passI[i,j,x] - bigM*(1-passI[i,j,x]) - bigM*(1-w[x,x1,j]);

subject to split3FixedJ {i in freeF, j in fixedF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x2,j]==1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min*passJ[i,j,x] - bigM*(1-passJ[i,j,x]) - bigM*(1-w[x,x1,i]);
subject to split3_1FixedJ {i in freeF, j in fixedF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,j] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min*passJ[i,j,x] - bigM*(1-passJ[i,j,x]) - bigM*(1-w[x,x2,i]);



subject to split4 {i in freeF, j in freeF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x,x1,i]+w[x,x2,j] == 2) ==> (t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min * (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);
subject to split4_1 {i in freeF, j in freeF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
(w[x,x1,j]+w[x,x2,i] == 2) ==> (t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min * (1-passFirst[i,j,x]) - bigM*passFirst[i,j,x]);

subject to split4Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and i<>j and (wFixed[x,x1,i]+wFixed[x,x2,j] == 2) and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*D/v_min * (1-passFixed[i,j,x]) - bigM*passFixed[i,j,x];
subject to split4_1Fixed {i in fixedF, j in fixedF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and i<>j and (wFixed[x,x1,j]+wFixed[x,x2,i] == 2) and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*D/v_min * (1-passFixed[i,j,x]) - bigM*passFixed[i,j,x];

subject to split4FixedI {i in fixedF, j in freeF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,i] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min * (1-passI[i,j,x]) - bigM*passI[i,j,x] - bigM*(1-w[x,x2,j]);
subject to split4_1FixedI {i in fixedF, j in freeF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x2,i] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear_fixed[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min * (1-passI[i,j,x]) - bigM*passI[i,j,x] - bigM*(1-w[x,x1,j]);

subject to split4FixedJ {i in freeF, j in fixedF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,j] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*D/v_min * (1-passJ[i,j,x]) - bigM*passJ[i,j,x] - bigM*(1-w[x,x1,i]);
subject to split4_1FixedJ {i in freeF, j in fixedF, x in V,(x,x1) in E, (x,x2) in E: (x,x1,x2) in conflictsNodes and (wFixed[x,x1,j] == 1) and i<>j and x1<>x2 and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))}:
t_ear[i,x]- t_lat_fixed[j,x]>=angle[x,x1,x2]*D/v_min * (1-passJ[i,j,x]) - bigM*passJ[i,j,x] - bigM*(1-w[x,x2,i]);


subject to driftEar:
t_ear_fixed[drifted_flight,drifted_wp] >= drifted_t_ear_fix;
#TODO controllare se devo inserire t_ear o t_lat
subject to driftLat:
t_lat_fixed[drifted_flight,drifted_wp] <= drifted_t_lat_fix;
/*subject to driftWP:
sum{(x,drifted_wp) in E} w[x,drifted_wp,drifted_flight] = 1;
*/

subject to abs1{f in freeF}:
abs_t_ear[f] >= t_ear[f,e[f]] - t_hat_ear[f,e[f]];
subject to abs2{f in freeF}:
abs_t_ear[f] >= t_hat_ear[f,e[f]] - t_ear[f,e[f]];


subject to abs1Fixed{f in fixedF}:
abs_t_ear[f] >= t_ear_fixed[f,e[f]] - t_hat_ear[f,e[f]];
subject to abs2Fixed{f in fixedF}:
abs_t_ear[f] >= t_hat_ear[f,e[f]] - t_ear_fixed[f,e[f]];

subject to cutL1{i in fixedF, j in freeF, x in V: (i,x) in fixedFNodes}:
passI[i,j,x] + passJ[j,i,x] = 1;

subject to cutL2{i in freeF, j in fixedF, x in V: (j,x) in fixedFNodes}:
passI[j,i,x] + passJ[i,j,x] = 1;


#objective 49
#minimize z: sum{i in F,x in V, y in V: (x,y) in E} w[x,y,i];
#objective 50
minimize opt: sum{i in F} abs_t_ear[i];


/*
subject to trail1{i in F,j in F, x in V, y in V:(x,y) in E}:
(w[x,y,i]+ w[x,y,j]=2) ==> ((v[i,y]*(t[j,x]-t[i,x])>= D) or
(v[j,y]*(t[i,x]-t[j,x]) >=D));
subject to trail2{i in F,j in F, x in V, y in V:(x,y) in E}:
(w[x,y,i]+ w[x,y,j]=2) ==> ((v[i,y]*(t[j,y]-t[i,y])>= D) or (v[j,y]*(t[i,y]-t[j,y]) >=D));
subject to merge{i in F,j in F, x in V, x1 in V, x2 in V:(x1,x) in E and (x2,x) in E}:
(w[1,x,i]+ w[x2,x,j]=2) ==> ((t[j,x]-t[i,x]>=angleM[i,j,x]* D/v[j,x]) or (t[i,x]-t[j,x] >=
angleM[i,j,x]*D/v[i,x]));
subject to diver{i in F,j in F, x in V, x1 in V, x2 in V:(x,x1) in E and (x2,x) in E}:
(w[x,x1,i]+ w[x2,x,j]=2) ==> ((t[j,x]-t[i,x]>= anglePM[i,j,x]*(D/v[j,x]+ D/v[i,x1])) or
(t[i,x]-t[j,x] >= anglePM[i,j,x]*(D/v[i,x]+ D/v[j,x1])));
subject to split{i in F,j in F, x in V, x1 in V, x2 in V:(x,x1) in E and (x,x2) in E}:
(w[x,x1,i]+ w[x,x2,j]=2) ==> ((t[j,x]-t[i,x]>= angleP[i,j,x]* D/v[i,x1]) or (t[i,x]-t[j,x]
>=angleP[i,j,x]*D/v[j,x2]));
*/