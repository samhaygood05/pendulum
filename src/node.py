from math import *
from scipy.constants import g


class Connector:
    def __init__(self, length, k, connector_type='rigid'):
        self.connector_type = connector_type
        self.length = length
        self.k = k

    def __repr__(self):
        return f'Connector({self.length}, {self.k}, {self.connector_type})'


class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def normal(self):
        return Vector(-self.y, self.x)

    def rotate(self, angle):
        return Vector(self.x * cos(angle) - self.y * sin(angle), self.x * sin(angle) + self.y * cos(angle))

    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, other):
        return Vector(self.x * other, self.y * other)

    def __truediv__(self, other):
        return Vector(self.x / other, self.y / other)

    def __neg__(self) -> 'Vector':
        return Vector(-self.x, -self.y)

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def angle(self):
        return atan2(self.x, -self.y)

    def normalize(self):
        return self / sqrt(self.dot(self))

    def __repr__(self):
        return f'({self.x}, {self.y})'

    def coords(self):
        return self.x, -self.y


class Node:
    def __init__(self, mass, movable=True, color=None):
        if color is None and movable:
            self.color = (255, 255, 255)
        elif color is None and not movable:
            self.color = (255, 0, 0)
        else:
            self.color = color
        self.mass = mass
        self.movable = movable
        self.parent = None
        self.parent_connector = Connector(1, 0)
        self.children = []
        self.depth = 0

    def __repr__(self):
        return f'NodeBase({self.mass}, {self.movable}, {self.parent}, {self.parent_connector})'

    def give_parent(self, parent_node, parent_connector):
        self.parent = parent_node
        self.parent_connector = parent_connector
        self.depth = parent_node.depth + 1
        parent_node.children.append(self)


class PhysicsNode(Node):
    def __init__(self, position, mass, velocity=Vector(0, 0), movable=True, color=None):
        super().__init__(mass, movable, color)
        self.position = position
        self.velocity = velocity

    def __copy__(self):
        return PhysicsNode(self.position, self.mass, self.velocity, self.movable, self.color)

    def angle(self):
        if self.parent is None:
            return 0
        else:
            return (self.parent.position - self.position).angle()

    def parent_direction(self):
        if self.parent is None:
            return Vector(0, 0)
        else:
            return self.parent_disp().normalize()

    def parent_disp(self):
        if self.parent is None:
            return Vector(0, 0)
        else:
            return self.parent.position - self.position

    def repair_position(self):
        if self.parent is not None and self.parent_connector.connector_type == 'rigid':
            self.position = self.parent.position - self.parent_direction() * self.parent_connector.length

    def net_force(self):
        net_force = self.weight() + self.child_force() + self.parent_force()
        if self.parent_connector.connector_type == 'rigid':
            return Vector(0, net_force.dot(-self.parent_direction().normal()) * sqrt(abs(net_force.dot(-self.parent_direction().normal()))))
        else:
            return net_force

    def update_velocity(self, dt, resistance=0):
        if self.movable:
            if self.parent_connector.connector_type == 'rigid':
                self.velocity += self.net_force() * dt / self.mass
            self.velocity += self.net_force() / self.mass * dt
            self.velocity -= self.velocity * resistance * dt

    def update_position(self, dt):
        if self.movable:
            if self.parent_connector.connector_type == 'rigid':
                angular_velocity = self.velocity.y / self.parent_connector.length
                self.position = -self.parent_disp().rotate(angular_velocity * dt) + self.parent.position
                self.repair_position()
            else:
                self.position += self.velocity

    def weight(self):
        return Vector(0, -self.mass * g)

    def child_force(self):
        force = Vector(0, 0)
        for child in self.children:
            force -= child.parent_force()
        return force

    def parent_force(self) -> 'Vector':
        if self.parent is None:
            return Vector(0, 0)
        elif self.parent_connector.connector_type == 'rigid':
            net_child_force = Vector(0, 0)
            if self.children:
                for child in self.children:
                    net_child_force += child.net_force()
            return self.parent_direction() * (net_child_force.dot(self.parent_direction()) - self.weight().dot(self.parent_direction()))
        elif self.parent_connector.connector_type == 'spring':
            return (-self.parent_direction() * self.parent_connector.length + self.parent_disp()) * self.parent_connector.k

    def connector_color(self):
        if self.parent_connector.connector_type == 'spring':
            vec_disp = self.parent_direction() * self.parent_connector.length + self.parent_disp()
            force = self.parent_connector.k * vec_disp.dot(self.parent_direction())
            if force < 0:
                norm_factor = 0
            else:
                norm_factor = 1
            # else:
            #     norm_factor = (force + 10) / 20
            return 255 * norm_factor, 255 * (1 - norm_factor), 0
        else:
            return 255, 255, 255

    def strip_physics(self):
        node = Node(self.mass, self.movable)
        node.give_parent(self.parent, self.parent_connector)
        return node

    def __repr__(self):
        return f'Node({self.position}, {self.mass}, {self.velocity}, {self.movable}, {self.parent}, ' \
               f'{self.parent_connector}, {self.depth})'


class System:

    def __init__(self):
        self.nodes = []

    def root_node(self):
        self.append_node(Node(False))

    def append_node(self, node, parent_node=None, parent_connector=Connector(1, 0)):
        if parent_node is not None:
            node.give_parent(parent_node, parent_connector)
        self.nodes.append(node)

    def layer(self, depth):
        return [node for node in self.nodes if node.depth == depth]

    def max_depth(self):
        return max([node.depth for node in self.nodes])


class PhysicsSystem(System):
    def __init__(self, dt, resistance=0):
        super().__init__()
        self.target = None
        self.last_position = None
        self.initial_conditions = None
        self.resistance = resistance
        self.dt = dt
        self.steps = 0

    def __copy__(self):
        new_system = PhysicsSystem(self.dt)
        new_system.nodes = [node.__copy__() for node in self.nodes]
        new_system.target = self.target
        new_system.last_position = self.last_position
        new_system.initial_conditions = self.initial_conditions
        new_system.steps = self.steps
        return new_system

    def root_node(self, position=Vector(0, 0), color=None):
        self.append_node(PhysicsNode(position, 1, Vector(0, 0), False, color))

    def elapsed_time(self):
        return self.steps * self.dt

    def get_initial_conditions(self):
        self.initial_conditions = [(node.position, node.velocity) for node in self.nodes]

    def set_target(self, target):
        self.target = target

    def repair_positions(self):
        for node in self.nodes:
            node.repair_position()

    def update(self):
        dx = Vector(0, 0)
        for i in range(self.max_depth(), -1, -1):
            for node in self.layer(i):
                node.update_velocity(self.dt, self.resistance)
        for node in self.nodes:
            node.update_position(self.dt)
            # node.repair_position()
        self.steps += 1

    def strip_physics(self):
        system = System()
        for node in self.nodes:
            system.append_node(node.strip_physics())
        return system

    def __str__(self):
        return f'{self.nodes}'

    def __repr__(self):
        return f'{self.nodes}'
