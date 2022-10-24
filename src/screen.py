import pygame

from node import PhysicsSystem, PhysicsNode, Vector, Connector


class Screen:
    def __init__(self, node_systems, scale):
        self.node_systems = node_systems
        self.scale = scale
        self.screen = pygame.display.set_mode((1000, 800))

    def draw(self, draw_forces=False):
        self.screen.fill((0, 0, 0))
        for node_system in self.node_systems:
            for node in node_system.nodes:
                if node.parent:
                    pygame.draw.line(self.screen, node.connector_color(), (node.parent.position * self.scale).coords(), (node.position * self.scale).coords())
                pygame.draw.circle(self.screen, node.color, (node.position * self.scale).coords(), 5)
                if draw_forces:
                    pygame.draw.line(self.screen, (255, 0, 0), (node.position * self.scale).coords(), ((node.position * self.scale) + node.net_force()).coords())
            pygame.display.flip()

    def update(self, draw_forces=False):
        for node_system in self.node_systems:
            node_system.update()
        self.draw(draw_forces)

    def run(self, draw_forces=False):
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return
            self.update(draw_forces)


if __name__ == '__main__':
    system1 = PhysicsSystem(0.01)
    system1.root_node(Vector(9, -7), (255, 0, 0))
    system1.append_node(PhysicsNode(Vector(8, -8), 1, Vector(0, 0)), system1.nodes[0], Connector(1, 10, 'spring'))
    system1.append_node(PhysicsNode(Vector(9, -8), 1, Vector(0, 0)), system1.nodes[1], Connector(1, 10, 'spring'))
    screen = Screen([system1], 50)
    screen.run(True)

