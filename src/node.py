from math import *

from scipy.constants import g


class Connector:
    def __init__(self, length, k, connector_type='rigid'):
        self.connector_type = connector_type
        self.length = length
        self.k = k


class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, other):
        return Vector(self.x * other, self.y * other)

    def __truediv__(self, other):
        return Vector(self.x / other, self.y / other)

    def __neg__(self):
        return Vector(-self.x, -self.y)

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def angle(self):
        return atan2(self.x, -self.y)

    def normalize(self):
        return self / sqrt(self.dot(self))

    def __str__(self):
        return f'({self.x}, {self.y})'


class Node:
    def __init__(self, position, mass, velocity=Vector(0, 0), movable=True, parent=None, parent_connector=Connector(1, 0)):
        self.position = position
        self.velocity = velocity
        self.mass = mass
        self.movable = movable
        self.parent = parent
        parent.children.append(self)
        self.children = []
        self.parent_connector = parent_connector
        self.layer = parent.layer + 1 if parent is not None else 0

    def angle(self):
        if self.parent is None:
            return 0
        else:
            return (self.parent.position - self.position).angle()

    def parent_direction(self):
        if self.parent is None:
            return Vector(0, 0)
        else:
            return (self.parent.position - self.position).normalize()

    def net_force(self):
        return self.weight() + self.parent_force() + self.child_force()

    def update(self, dt):
        if self.movable:
            self.velocity += self.net_force() / self.mass * dt
            self.position += self.velocity * dt

    def weight(self):
        return Vector(0, -self.mass * g)

    def child_force(self):
        if len(self.children) == 0:
            return Vector(0, 0)
        else:
            return sum([-child.parent_force() for child in self.children])

    def parent_force(self):
        if self.parent is None:
            return Vector(0, 0)
        elif self.parent_connector.connector_type == 'rigid':
            net_child_force = sum([child.net_force() for child in self.children])
            return self.parent_direction() * net_child_force.dot(self.parent_direction())
        elif self.parent_connector.connector_type == 'spring':
            pass
            return self.parent_connector.k * (self.parent.position - self.position)


class System:
    def __init__(self, nodes=None):
        if nodes is None:
            nodes = []
        self.nodes = nodes

    def append_node(self, node):
        self.nodes.append(node)

    def layer(self, layer):
        return [node for node in self.nodes if node.layer == layer]

    def max_layer(self):
        return max([node.layer for node in self.nodes])

    def update(self, dt):
        for i in range(self.max_layer(), -1, -1):
            for node in self.layer(i):
                node.update(dt)

    def __str__(self):
        return f'{self.nodes}'

    def __repr__(self):
        return f'{self.nodes}'