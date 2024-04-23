# -*- coding: utf-8 -*-
"""
Created on Wed Apr 17 06:18:37 2024

@author: JRFRIZZELL
"""

from geopy.distance import great_circle
import time
from pulp import lpSum, LpProblem, LpMaximize, LpSolverDefault
from pulp import LpVariable, LpContinuous, LpBinary
from amply import Amply
# import ro_utils

data = Amply("""
set parts;
param prodRate{parts, machines};
param yields{parts};
""")

MAX_ROS = int(input("How many offices should the model output? "))

smallring = 60
bigring = 120
ROdist = bigring
DIV_FACTOR = 9000

start_time = time.time()

cbsa = {}
mkt_values = []
numC = 0
for line in open('CBSA_final.txt'):
    Ccode, Cname, lat, long, scaledAcc = line.split("\t")
    long = long.strip("\n")
    locC = (float(lat), float(long))
    scaledAcc = int(scaledAcc.strip().replace("\n","")) / DIV_FACTOR
    mkt_values.append(scaledAcc)
    Cname = Cname.strip("\"")
    cbsa[numC] = [Cname, locC, scaledAcc, int(Ccode)]
    numC += 1    

MEPS = {}
numM = 0
for line in open('MEPS_Locations.txt'):
    name, lat, long = line.split("\t")
    long = long.strip("\n")
    locM = (float(lat), float(long))
    MEPS[int(numM)] = [name.strip("\""), locM, lat, long]
    numM += 1

Units = {}
numU = 0
for line in open('CG_Units.txt'):
    name, bodies, lat, long = line.split("\t")
    long = long.strip("\n")
    locU = (float(lat), float(long))
    Units[(int(numU))] = [
        name.strip("\""), locU, lat, long, int(bodies)]
    numU += 1

PropROs = {}
numR = 0
for line in open('Prop_ROs.txt'):
    name, lat, long = line.split("\t")
    long = long.strip("\n")
    locR = (float(lat), float(long))
    PropROs[int(numR)] = [name.strip("\""), locR, lat, long]
    numR += 1
    
distCBSA = {}
IR_mkts = {}
OR_mkts = {}
for i in PropROs:
    IR_mkts[i] = list()
    OR_mkts[i] = list()
    for j in cbsa:
        location1 = PropROs[i][1]
        location2 = cbsa[j][1]
        dist = int(great_circle(location1, location2).miles)
        distCBSA[(i,j)] = dist
        if distCBSA[(i,j)] < bigring:
            if distCBSA[(i,j)] < smallring:
                IR_mkts[i].append(int(j))
            else:
                OR_mkts[i].append(int(j))


markets = range(len(cbsa))
roffices = range(len(PropROs))
AR_mkts = {i:list(set(IR_mkts[i]+OR_mkts[i])) for i in IR_mkts.keys() }

# RO sets relative to markets
def make_inv_dict(input_dict):
    inv_dict = {}
    for k,v in input_dict.items():
        for x in v:
            inv_dict.setdefault(x, []).append(k)
    return inv_dict

IR_ros = make_inv_dict(IR_mkts)
OR_ros = make_inv_dict(OR_mkts)
AR_ros = make_inv_dict(AR_mkts)

d = dict(zip(markets, mkt_values))
c = 20  # recruits accessed per recruiter
w_or = 0.5  # outer ring recruiter reduction factor
# attr = {1: 1, 2: 1, 3:1}  # attraction factor for RO, larger is more
attr = {}
for i in range(len(d)):
    attr[i] = 1
max_num_ROs = MAX_ROS  # maximum num of ROs to open

# mu_lo = 3
# mu_hi = 12
mu = 7   # fixing # of recruiters per office for now
M = 5*c*mu  # larger than total supply from an RO
epsilon = .001  # small # to enforce delta_i = 0 if sum(x_i_j over j)=0


# Set the verbosity level
LpSolverDefault.msg = 0  # Set to 0 for minimal output, 1 for normal, -1 for silent


# data.load_file(open(file_name))
ro_opt_model = LpProblem('Recruiting_Office_Placement_&_Sizing',LpMaximize)

# Variables
x = LpVariable.dicts('x',(roffices, markets),0,cat=LpContinuous) 
# mu = LpVariable.dicts('mu',roffices,0,cat=LpInteger)
delta = LpVariable.dicts('delta',roffices,0,cat=LpBinary)

# Objective function
ro_opt_model += lpSum(x[i][j] for i in roffices for j in AR_mkts[i]) 

# Constraints
for i in roffices:
    ro_opt_model += lpSum(x[i][j] for j in AR_mkts[i]) <= c * mu,   \
                    f'RO_{i} staffing supply capacity'
    # ro_opt_model += mu[i] >= mu_lo * delta[i], f'RO_{i} staff lower bound'
    # ro_opt_model += mu[i] <= mu_hi * delta[i], f'RO_{i} staff upper bound'

for j in markets:
    tmp_sum = 0
    if j in IR_ros.keys():   # if this mkt is in IR of an RO
        tmp_sum += lpSum(attr[i]*x[i][j] for i in IR_ros[j])
    if j in OR_ros.keys():   # if this mkt is in OR of an RO
        tmp_sum += w_or * lpSum(attr[i]*x[i][j] for i in OR_ros[j])
    if tmp_sum:
        ro_opt_model += tmp_sum <= d[j],  \
          f'RO accessions do not exceed market {j} demand'
    
# add indicator variable for RO constraint
for i in roffices:
    ro_opt_model += delta[i] >= lpSum(x[i][j] for j in AR_mkts[i]) / M
    
    ro_opt_model += delta[i] <= lpSum(x[i][j] for j in AR_mkts[i]) / epsilon
    
ro_opt_model += lpSum(delta[i] for i in roffices) <= max_num_ROs, \
    'Constraint total number of ROs to open'  
    
#print(ro_opt_model)

ro_opt_model.solve()


# print()
# for i in roffices:
#     print(f'Of a total supply of {c*mu} units:')
#     for j in markets:
#         print(f'RO{i} ships {x[i][j].varValue} units to MKT{j}',end=' ')#,extra_str)
#         ro_utils.adjusted_flow(
#             x[i][j].varValue,attr[i],w_or*(j in OR_mkts[i]))
#         print()
    
# print()
# for j in markets:
#     print(f'Of a total demand of {d[j]} units:')
#     for i in roffices:
#         print(f'MKT{j} receives {x[i][j].varValue} units from RO{i}')
#         ro_utils.adjusted_flow(
#             x[i][j].varValue,attr[i],w_or*(j in OR_mkts[i]))
#     print()

    
print('The total # of ROs opened is',lpSum(delta[i] for i in roffices).value())


off_count_dummy = 1
acc_count_dummy = 0
for variable in ro_opt_model.variables():
    if variable.varValue > 0:
        off_count_dummy += 1
        acc_count_dummy += variable.varValue
        #print(f"{variable.name}: {variable.varValue}")

ro_dict = {}
for variable in ro_opt_model.variables():
    if variable.varValue > 0 and variable.name[0] != "d":
        ro = int(variable.name[2:].split("_")[0]) 
        if ro in ro_dict:
            ro_dict[ro] += variable.varValue
        else:
            ro_dict[ro] = variable.varValue

final_names = {}
for ro in ro_dict:
    ro_dict[ro] = round(ro_dict[ro],3)
    #print(ro_dict[ro])
    ro_name = PropROs[ro][0]
    final_names[ro_name] = ro_dict[ro]


sort_names = (dict(sorted(final_names.items(), 
                          key=lambda item: item[1], reverse=True)))   
print("\n\n",sort_names)
print("\nTotal ROs:",len(ro_dict))
print("Pairs:", off_count_dummy, "\nRecruits:", int(acc_count_dummy))

end_time = time.time()
print(str(round(end_time-start_time, 2)) 
      + " seconds have passed. \n")
