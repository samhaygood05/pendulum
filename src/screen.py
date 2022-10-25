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
                    if node.parent_connector.connector_type == 'spring':
                        pygame.draw.line(self.screen, (255, 0, 0), (node.position * self.scale).coords(), ((node.position * self.scale) + node.net_force()).coords())
                    else:
                        pygame.draw.line(self.screen, (255, 0, 0), (node.position * self.scale).coords(), ((node.position * self.scale) - node.parent_direction().normal() * node.net_force().y).coords())
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
    system = PhysicsSystem(0.001, 0)
    system.root_node(Vector(7, -1))
    system.append_node(PhysicsNode(Vector(8, -1), 1, Vector(0, 0)), system.nodes[0], Connector(1, 200))
    system.append_node(PhysicsNode(Vector(9, -1), 1, Vector(0, 0)), system.nodes[1], Connector(1, 20, 'spring'))

    system.repair_positions()
    system.get_initial_conditions()
    screen = Screen([system], 50)
    screen.run(True)

