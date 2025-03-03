#data
param nn;																		#number of nodes
set F;																            #set of flights
set AP within F default {};
set NC within F default {};
set V:= 0..nn-1;																#set of nodes
set E within {V cross V};														#arcs
param s{F};																		#starting point for each flight
param e{F};																		#ending poitn for each flight
param d{E}>=0 default 0;											            #distance for each pair of nodes
param v_max;
param v_min;
param dMCF{F,E} >=0 default 0;
param bigM:= 1000;	                                                            #bigM for linearizing purpose
param W:= 1000;																	#weight for the objective function
set conflictsNodes within {x in V, x1 in V, x2 in V: x1<>x2 and ((x,x1) in E or (x1,x) in E) and ((x,x2) in E or (x2,x) in E)};
param angle{conflictsNodes} ;
param D;																		# safety distance
param t_hat_ear{F,V} default 0;
param t_hat_lat{F,V} default 0;
param tini;
set inFlight;
param w{(i,j) in E,F} binary;											#flight f pass through arc i,j
set PathAP within {AP cross E} default {};
set PathNC within {NC cross E} default {};
set nodesAP = setof{(f,x,y) in PathAP} x union setof{(f,x,y) in PathAP} y;
#param and set built

#set mercedes for inflights flights
#TODO check here for names and possibile corrections
set passedF within {F cross E} default {};
set passedFN := {setof {(f,x,y) in passedF} (f,x)} union {setof {(f,x,y) in passedF} (f,y)};
set passedNode within passedFN:= {(f,x) in passedFN: f in NC or (f in inFlight and t_hat_ear[f,x] <= tini)};
set notPassedNode within passedFN := passedFN diff passedNode;
set firstNotPassed within notPassedNode:={(f,x) in notPassedNode: f in inFlight and 
    not exists {(j,r) in notPassedNode} (j == f and t_hat_ear[j,r] < t_hat_ear[f,x])};
set timeFixed = firstNotPassed union passedNode;

#set used in heuristic
set FV = {(f,v) in (F cross V) : 
             exists {(i,j) in E: (j == v or i == v)} w[i,j,f] == 1};

var t_down  {(i,x) in FV: x<>s[i]} >= 0;
var t_up    {(i,x) in FV: x<>s[i]} >= 0;
var t_ear   {FV}  >=0 ;													#variable time
var t_lat   {FV}  >=0 ;                                                 #variable time                                  
var t_ear_start{F} >=0 integer;			                                #variable to make the first flight starting at integer time													
var l{i in F, j in F, x in V: (i,x) in FV and (j,x) in FV and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))} binary;

var abs_t_ear{F diff (AP union NC)} >= 0;									#variable to calculate the absolute value of t_ear for ending node
var delayPriority{AP} integer >= 0;												#delay for priority flight
#heuristic var
var wPath{(i,j) in E, F diff (AP union NC)} binary;	

#sets to avoid degenerate constrains
set differentAngle=setof{(x,x1,x2) in conflictsNodes} angle[x,x1,x2];

set allConf={i in F, j in F, x in V, differentAngle:i<>j and  (i,x) in FV and (j,x) in FV and (t_hat_ear[i,x] < t_hat_ear[j,x] or (t_hat_ear[i,x] == t_hat_ear[j,x] and i<j))};

set diver1Conf={(i,j,x,dA) in allConf,(x,x1) in E, (x2,x) in E: i<>j and w[x,x1,i] + w[x2,x,j]== 2 and (x,x1,x2) in conflictsNodes and angle[x,x1,x2]==dA};
set diver2Conf={(i,j,x,dA) in allConf,(x,x1) in E, (x2,x) in E: i<>j and w[x,x1,j] + w[x2,x,i]== 2 and (x,x1,x2) in conflictsNodes and angle[x,x1,x2]==dA};
set diver1SetOf=setof{(i,j,x,dA,x1,x2) in diver1Conf} (i,j,x,dA);
set diver2SetOf=setof{(i,j,x,dA,x1,x2) in diver2Conf} (i,j,x,dA);

set trailConf={(i,j,x,dA) in allConf, (i,j,y,dA) in allConf: (x,y) in E and i<>j and w[x,y,i]+w[x,y,j]==2};
#set trail2Conf={i in F,j in F, x in V,dA in differentAngle, (i,j,y,dA) in allConf: (x,y) in E and  i<>j and w[x,y,i]+w[x,y,j]==2};
set trail1SetOf=setof{(i,j,x,dA,y) in trailConf} (i,j,x,dA);
set trail2SetOf=setof{(i,j,x,dA,y) in trailConf} (i,j,y,dA);

set splitConf={(i,j,x,dA) in allConf,(x,x1) in E, (x,x2) in E: i<>j and x1<>x2 and w[x,x1,i]+w[x,x2,j]==2 and (x,x1,x2) in conflictsNodes and angle[x,x1,x2]==dA};
#set split2Conf={(i,j,x,dA) in allConf,(x,x1) in E, (x,x2) in E: i<>j and x1<>x2 and w[x,x1,j]+w[x,x2,i]==2 and (x,x1,x2) in conflictsNodes and angle[x,x1,x2]==dA};
set splitSetOf=setof{(i,j,x,dA,x1,x2) in splitConf} (i,j,x,dA);
#set split2SetOf=setof{(i,j,x,dA,x1,x2) in split2Conf} (i,j,x,dA);

set mergeConf={(i,j,x,dA) in allConf,(x1,x) in E, (x2,x) in E: x1<>x2 and i<>j and w[x1,x,i]+w[x2,x,j]==2  and (x,x1,x2) in conflictsNodes and angle[x,x1,x2]==dA};
#set merge2Conf={(i,j,x,dA) in allConf,(x1,x) in E, (x2,x) in E: x1<>x2 and i<>j and w[x1,x,j]+w[x2,x,i]==2  and (x,x1,x2) in conflictsNodes and angle[x,x1,x2]==dA};
set mergeSetOf=setof{(i,j,x,dA,x1,x2) in mergeConf} (i,j,x,dA);
#set merge2SetOf=setof{(i,j,x,dA,x1,x2) in merge2Conf} (i,j,x,dA);

set diver1 = setof{(i,j,x,dA,x1,x2) in diver1Conf} (i,j,x,x1,x2);
set diver2 = setof{(i,j,x,dA,x1,x2) in diver2Conf diff diver1Conf} (i,j,x,x1,x2);
set merge  = setof{(i,j,x,dA,x1,x2) in mergeConf: (i,j,x,dA) not in (diver1SetOf union diver2SetOf)} (i,j,x,x1,x2);
set split  = setof{(i,j,x,dA,x1,x2) in splitConf: (i,j,x,dA) not in (diver1SetOf union diver2SetOf union mergeSetOf)} (i,j,x,x1,x2);
set trail1 = setof{(i,j,x,dA,y) in trailConf:(i,j,x,dA) not in (diver1SetOf union diver2SetOf union mergeSetOf union splitSetOf )} (i,j,x,y);
set trail2 = setof{(i,j,x,dA,y) in trailConf:(i,j,y,dA) not in (diver1SetOf union diver2SetOf union mergeSetOf union splitSetOf/*  union trail1SetOf */)} (i,j,x,y);

/*
set diver1 = setof{(i,j,x,dA,x1,x2) in diver1Conf} (i,j,x,x1,x2);
set diver2 = setof{(i,j,x,dA,x1,x2) in diver2Conf diff diver1Conf} (i,j,x,x1,x2);
set merge1 = setof{(i,j,x,dA,x1,x2) in merge1Conf: (i,j,x,dA) not in (diver1SetOf union diver2SetOf)} (i,j,x,x1,x2);
set merge2 = setof{(i,j,x,dA,x1,x2) in merge2Conf: (i,j,x,dA) not in (diver1SetOf union diver2SetOf union merge1SetOf)} (i,j,x,x1,x2);
set split1 = setof{(i,j,x,dA,x1,x2) in split1Conf: (i,j,x,dA) not in (diver1SetOf union diver2SetOf union merge1SetOf union merge2SetOf)} (i,j,x,x1,x2);
set split2 = setof{(i,j,x,dA,x1,x2) in split2Conf: (i,j,x,dA) not in (diver1SetOf union diver2SetOf union merge1SetOf union merge2SetOf union split1SetOf)} (i,j,x,x1,x2);
set trail1 = setof{(i,j,x,dA,y) in trailConf:(i,j,x,dA) not in (diver1SetOf union diver2SetOf union merge1SetOf union merge2SetOf union split1SetOf union split2SetOf)} (i,j,x,y);
set trail2 = setof{(i,j,x,dA,y) in trailConf:(i,j,y,dA) not in (diver1SetOf union diver2SetOf union merge1SetOf union merge2SetOf union split1SetOf union split2SetOf)} (i,j,x,y);
*/

#constrains

subject to startingPath{i in F diff (AP union NC)}:
sum{(x,s[i]) in E} wPath[x,s[i],i] - sum{(s[i],x) in E} wPath[s[i],x,i] = -1;
subject to finishingPath{i in F diff (AP union NC)}:
sum{(x,e[i]) in E} wPath[x,e[i],i] - sum{(e[i],x) in E} wPath[e[i],x,i] = 1;
subject to allPath{i in F diff (AP union NC), x in V :  x <> s[i] and x <> e[i]}:
sum{(x,y) in E} wPath[x,y,i]=sum{(y,x) in E} wPath[y,x,i];

#define t_lat and integer starting time
subject to afterprecalculated{f in F diff inFlight}:
tini <= t_ear_start[f];
subject to calculateLat{(i,x) in FV diff timeFixed: i not in AP}:
t_lat[i,x]=t_ear[i,x] + t_hat_lat[i,x] - t_hat_ear[i,x];
subject to startInt{f in F}:
t_ear_start[f] = t_ear[f,s[f]];

#define t_up and t_down
subject to limitT_down{(i,x) in FV diff timeFixed : x <> s[i]}:
t_down[i,x] <= t_ear[i,x];
subject to limitT_up{(i,x) in FV diff timeFixed: x <> s[i]}:
t_ear[i,x] <= t_up[i,x];
subject to defineT_down{(f,y) in FV diff timeFixed, (x,y) in E: y <> s[f] and w[x,y,f] == 1 and f not in (AP union NC)}:	
t_down[f,y] = t_ear[f,x] + d[x,y]/v_max;
subject to defineT_up{(f,y) in FV diff timeFixed,(x,y) in E : y <> s[f] and w[x,y,f]==1 and f not in (AP union NC)}: 		
t_up[f,y]=t_ear[f,x] + d[x,y]/v_min;


#conflicts
subject to trail13 {(i,j,x,y) in trail1}:
t_ear[j,x]-t_lat[i,x] >= D/v_min * l[i,j,x]- bigM*(1-l[i,j,x]);
subject to trail14 {(i,j,x,y) in trail1}:
t_ear[i,x]-t_lat[j,x] >= D/v_min * (1-l[i,j,x]) - bigM*l[i,j,x];

subject to trail23 {(i,j,x,y) in trail2}:
t_ear[j,y]-t_lat[i,y]>= D/v_min* l[i,j,y] - bigM*(1-l[i,j,y]);
subject to trail24 {(i,j,x,y) in trail2}:
t_ear[i,y]-t_lat[j,y]>= D/v_min * (1-l[i,j,y]) - bigM*l[i,j,y];

subject to merge3{(i,j,x,x1,x2) in merge}:
t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min * l[i,j,x] - bigM*(1-l[i,j,x]);
#subject to merge3_2{(i,j,x,x1,x2) in merge2}:
#t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min * l[i,j,x] - bigM*(1-l[i,j,x]);
subject to merge4{(i,j,x,x1,x2) in merge}:
t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min* (1-l[i,j,x]) - bigM*l[i,j,x];
#subject to merge4_2{(i,j,x,x1,x2) in merge2}:
#t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min* (1-l[i,j,x]) - bigM*l[i,j,x];

subject to diver3_1 {(i,j,x,x1,x2) in diver1}:
t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*2*D/v_min* l[i,j,x] - bigM*(1-l[i,j,x]);
subject to diver3_2 {(i,j,x,x1,x2) in diver2}:
t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*2*D/v_min* l[i,j,x] - bigM*(1-l[i,j,x]);
subject to diver4_1 {(i,j,x,x1,x2) in diver1}:
t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*2*D/v_min* (1-l[i,j,x]) - bigM*l[i,j,x];
subject to diver4_2 {(i,j,x,x1,x2) in diver2}:
t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*2*D/v_min* (1-l[i,j,x]) - bigM*l[i,j,x];

subject to split3 {(i,j,x,x1,x2) in split}:
t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min*l[i,j,x] - bigM*(1-l[i,j,x]);
#subject to split3_2 {(i,j,x,x1,x2) in split2}:
#t_ear[j,x]- t_lat[i,x]>=angle[x,x1,x2]*D/v_min*l[i,j,x] - bigM*(1-l[i,j,x]);
subject to split4{(i,j,x,x1,x2) in split}: 
t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min * (1-l[i,j,x]) - bigM*l[i,j,x];
#subject to split4_2{(i,j,x,x1,x2) in split2}: 
#t_ear[i,x]- t_lat[j,x]>=angle[x,x1,x2]*D/v_min * (1-l[i,j,x]) - bigM*l[i,j,x];

#constrain added for mercedes dat
subject to fixPassedEar{(f,x) in timeFixed}:
t_ear[f,x] = t_hat_ear[f,x];
subject to fixPassedLat{(f,x) in timeFixed}:
t_lat[f,x] = t_hat_lat[f,x];

#fixing already done paths
subject to fixPassedW{(f,x,y) in passedF : (f,y) in timeFixed}:
wPath[x,y,f]= 1;

subject to priority1{f in AP, x in nodesAP}:
t_ear[f,x] = t_hat_ear[f,x] + delayPriority[f];
subject to priority2{f in AP, x in nodesAP}:
t_lat[f,x] = t_hat_lat[f,x] + delayPriority[f];

subject to abs1{f in F diff (AP union NC)}:
abs_t_ear[f] >= t_ear[f,e[f]] - t_hat_ear[f,e[f]];
subject to abs2{f in F diff (AP union NC)}:
abs_t_ear[f] >= t_hat_ear[f,e[f]] - t_ear[f,e[f]];

#minimize UAM: sum {i in AP} W*delayPriority[i] + sum{i in F diff (AP union NC)} abs_t_ear[i];
minimize UAM: sum {i in AP} W*delayPriority[i] + sum{i in F diff (AP union NC)} abs_t_ear[i];
minimize MC: sum{f in F diff (AP union NC), (x,y) in E} wPath[x,y,f] * dMCF[f,x,y];

#l'ordine è obj, variabili, vincoli
problem path: MC, 
#var
    wPath, 
#constrains
    startingPath,finishingPath,allPath,
    fixPassedW;

problem conflicts: UAM,
#variables
    t_ear, t_lat, t_ear_start,
    t_down, t_up, l,
    abs_t_ear,delayPriority,
#constrains
    afterprecalculated, calculateLat, startInt,
    limitT_down, limitT_up, 
    defineT_down, defineT_up,
    trail13, trail14,
    trail23, trail24,
    merge3, #merge3_2,
    merge4, #merge4_2,
    diver3_1, diver3_2,
    diver4_1, diver4_2,
    split3, #split3_2,
    split4, #split4_2,
    fixPassedEar,fixPassedLat,
    abs1,abs2,
    priority1, priority2;

