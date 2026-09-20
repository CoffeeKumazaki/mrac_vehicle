from control.matlab import *
from linear_system import *
from vehicle_params import *
from vehicle_dynamics import *

# 車種のリスト
car_types = ['Prius', 'Blazer', 'Truck', 'Small', 'Van']

# 車種ごとのパラメータの辞書を作成し、辞書のリストに格納する
car_params = []
car_params.append({'mass': 1650.0, 'Iz': 2900.0, 'cf': 100000.0, 'cr': 200000.0, 'lf': 1.1, 'lr': 1.6})
car_params.append({'mass': 1727.0, 'Iz': 2867.0, 'cf': 94000.0, 'cr': 94000.0, 'lf': 1.17, 'lr': 1.42})
car_params.append({'mass': 5500.0, 'Iz': 20600.0, 'cf': 50000.0, 'cr': 130000.0, 'lf': 2.5, 'lr': 1.5})
car_params.append({'mass': 1180.0, 'Iz': 1945.0, 'cf': 90000.0, 'cr': 90000.0, 'lf': 1.05, 'lr': 1.48})
car_params.append({'mass': 3040.0, 'Iz': 10660.0, 'cf': 90000.0, 'cr': 90000.0, 'lf': 1.85, 'lr': 1.895})

# 車種ごとに伝達関数を計算する
vx = 20.0
for i in range(len(car_types)):
    # 車両のパラメータを取得
    modelParam = VehicleParam()
    modelParam.mass = car_params[i]['mass']
    modelParam.Iz = car_params[i]['Iz']
    modelParam.cf = car_params[i]['cf']
    modelParam.cr = car_params[i]['cr']
    modelParam.lf = car_params[i]['lf']
    modelParam.lr = car_params[i]['lr']

    A, B, C, D = linear_vehicle_model(modelParam, vx)
    mss = ss(A, B, C, D)
    print(car_types[i])
    print(tf(mss))
