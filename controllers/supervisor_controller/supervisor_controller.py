import controller
import numpy as np
from robot_controller.field import Field

if __name__ == '__main__':
    # data = '-1.077 0.05 -0.52 0.009 0.05 0.27 -0.669 0.05 0.079 0.803 0.05 -0.687 0.109 0.05 0.383 0.251 0.05 -0.496 0.602 0.05 0.711 0.102 0.05 -0.183'
    # data = '-0.78 0.05 0.604 -0.32 0.05 -0.58 -1.093 0.05 -0.339 0.214 0.05 0.244 0.218 0.05 -0.972 0.945 0.05 0.45 0.275 0.05 -1.103 0.137 0.05 -0.38'
    data = ''
    supervisor = controller.Supervisor()
    timestep = int(supervisor.getBasicTimeStep())
    nodes = []
    fields = []
    positions = []
    positions_set = False
    for i in range(1, 5):
        nodes.append(supervisor.getFromDef('Block_B' + str(i)))
        nodes.append(supervisor.getFromDef('Block_R' + str(i)))
    for node in nodes:
        fields.append(node.getField('translation'))

    cont_main_loop = False
    if not data:
        for i in range(len(fields)):
            while True:
                pos = np.random.uniform(-1.2 + 0.03, 1.2 - 0.03, size=(2,))
                for block in positions:
                    if np.linalg.norm(pos - block) < 0.07:
                        cont_main_loop = True
                        break
                if cont_main_loop:
                    cont_main_loop = False
                    continue
                if not Field.in_deposit_boxes(pos):
                    positions.append(pos)
                    break
        for i in range(len(positions)):
            positions[i] = [positions[i][0], 0.05, positions[i][1]]
    else:
        data = data.split(' ')
        for i in range(0, len(data), 3):
            if data[i] == '':
                continue
            positions.append([float(data[i]), float(data[i + 1]), float(data[i + 2])])

    while supervisor.step(timestep) != -1:
        if not positions_set:
            for i in range(len(fields)):
                fields[i].setSFVec3f(positions[i])
                nodes[i].resetPhysics()
            positions_set = True

    string = ''
    for pos in positions:
        string += ' '.join(map(lambda x: str(round(x, 3)), pos)) + ' '
    print(string)
