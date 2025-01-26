from utils.Point import Point

class ObjectiveManager:
    def __init__(self, agents, objectives):
        """
        Inicializa o gerenciador de objetivos.

        Args:
            agents (list): Lista de agentes disponíveis.
            objectives (list): Lista de objetivos a serem atribuídos.
        """
        self.agents = agents
        self.objectives = objectives
        self.assigned_targets = {agent.id: None for agent in agents}

    def calculate_cost(self, agent, objective):
        """
        Calcula o custo de atribuir um objetivo a um agente.
        O custo aqui é a distância euclidiana entre o agente e o objetivo.

        Args:
            agent: O agente para quem o custo será calculado.
            objective: O objetivo para o qual o custo será calculado.

        Returns:
            float: O custo calculado.
        """
        agent_position = Point(agent.robot.x, agent.robot.y)
        return agent_position.dist_to(objective)

    def assign_targets(self):
        """
        Atribui objetivos aos agentes usando uma abordagem gulosa.
        """
        unassigned_objectives = self.objectives.copy()
        for agent in self.agents:
            if len(unassigned_objectives) == 0:
                break

            # Encontrar o objetivo mais próximo para o agente
            closest_objective = min(
                unassigned_objectives,
                key=lambda obj: self.calculate_cost(agent, obj)
            )

            # Atribuir o objetivo e removê-lo da lista de objetivos não atribuídos
            self.assigned_targets[agent.id] = closest_objective
            unassigned_objectives.remove(closest_objective)

    def get_assigned_target(self, agent_id):
        """
        Retorna o objetivo atribuído a um agente.
        """
        return self.assigned_targets.get(agent_id, None)
