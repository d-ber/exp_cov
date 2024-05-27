#### DATA

param coverage_coeff; # coverage weight in objective function
param distance_coeff; # guard distance weight in objective function
param min_coverage; # minimum required coverage function, 0 <= min_coverage <= 1
param nW; # witness number (polygon points)
param nG; # guard number (robot poses)
set Witnesses := 1..nW; # witness set
set Guards := 1..nG; # guard set
param guard_cost {Guards}; # guard choice cost
param coverage {Witnesses, Guards}; # coverage[w, g] = 1 if guard g covers witness w, else 0
param distance {Guards, Guards}; # distance[g1, g2] = distance between g1 and g2


#### VARIABLES

var z; # objective var, total cost
var guard_choice {Guards} binary; # guard_choice[g] = 1 if g is chosen, else 0
var covered {Witnesses} binary; # covered[w] = 1 if w is covered, else 0
var arc_choice {Guards, Guards} binary; # arc_choice[g1, g2] = 1 if (g1, g2) arc is in the MST
var root {Guards} binary; # root[g] = 1 if g is the MST root (used for flow)
var flow {Guards, Guards} >= 0; # flow from g1 to g2
var vertex_n >= 0; # number of vertexes in MST

# Minimize total cost
minimize Total_Cost: 
    z;

#### CONSTRAINTS

# Define total cost as coeff_coverage*coverage + coeff_distanze*distance
s.t. Choice_Cost:
    z = coverage_coeff * (sum {g in Guards} guard_cost[g] * guard_choice[g]) + distance_coeff * (sum {g1 in Guards, g2 in Guards: g1<>g2} arc_choice[g1, g2] * distance[g1, g2]);

# Each witness must be covered by at least one guard, unless it is disabled
s.t. Coverage_Witnesses {w in Witnesses}:
    sum {g in Guards: coverage[w, g]} guard_choice[g]  >= 1 * covered[w];

# At least min_coverage*nW witnesses must be covered
s.t. Minimum_Coverage:
    (sum {w in Witnesses} covered[w]) / nW >= min_coverage;

## Minimum Spanning Tree Constraints (MST)
# Vertex num definition
s.t. Vertex_Num:
    vertex_n = sum {g in Guards} guard_choice[g];

# Only arcs between chosen guards can be chosen
s.t. Arcs_on_chosen {g1 in Guards, g2 in Guards: g1<>g2}:
    arc_choice[g1, g2] + arc_choice[g2, g1]  <= guard_choice[g1] * guard_choice[g2];

# There is a single root
s.t. One_Root:
    sum {r in Guards} root[r] = 1;

# The root is a chosen guard
s.t. Chosen_Root:
    sum {r in Guards} root[r] * guard_choice[r] = 1;

# Total arc number is vertex number - 1
s.t. Arc_Total:
    sum {g1 in Guards, g2 in Guards: g1<>g2} arc_choice[g1, g2] = vertex_n - 1;

# Flow from root is vertex number - 1
s.t. Root_Flow:
    (sum {r in Guards, g2 in Guards: g2<>r} flow[r, g2] * root[r]) = vertex_n - 1;

# Each non root vertex consumes 1 flow
s.t. Consumo_flow {g in Guards}:
    (1-root[g]) * guard_choice[g] * ((sum {g1 in Guards: g<>g1} flow[g1, g]) - (sum {g2 in Guards: g<>g2} flow[g, g2])) = (1-root[g]) *  guard_choice[g];

# Flow only possible if arc in tree, and upper limit of vertex number - 1
s.t. flow_in_albero {g1 in Guards, g2 in Guards: g1<>g2}:
    flow[g1, g2] <= (vertex_n - 1) * arc_choice[g1, g2];