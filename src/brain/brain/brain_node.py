#!/usr/bin/env python3
"""
ROS2 Node for playing Tic-Tac-Toe with MENACE AI.
Integrates with perception and manipulation packages.
"""

import numpy as np
import pygame
import os
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Int32MultiArray, Int32, Bool
from geometry_msgs.msg import Pose2D
from helper.msg import GridPose
from helper.action import DrawShape
from ament_index_python.packages import get_package_share_directory


def encode_board_base3(board_array: np.ndarray) -> int:
    """
    Converts a 3x3 board array (with values -1, 0, 1) to a base-3 integer.
    """
    board_flat = board_array.flatten()
    mapped_board = np.where(board_flat == -1, 2, np.where(board_flat == 0, 0, 1))
    base_3 = 0
    for i in range(9):
        base_3 += mapped_board[i] * (3**i)
    return base_3


class TicTacToe:
    """Game Engine."""

    def __init__(self):
        self.board = np.zeros((3, 3), dtype=int)  # 0=empty, 1=X, -1=O
        self.current_player = 1  # X starts
        self.game_over = False
        self.winner = None

    def reset(self):
        self.board = np.zeros((3, 3), dtype=int)
        self.current_player = 1
        self.game_over = False
        self.winner = None

    def get_legal_moves(self) -> List[Tuple[int, int]]:
        return [(i, j) for i in range(3) for j in range(3) if self.board[i, j] == 0]

    def make_move(self, row: int, col: int) -> bool:
        if self.game_over or self.board[row, col] != 0:
            return False

        self.board[row, col] = self.current_player
        self.check_game_over()

        if not self.game_over:
            self.current_player *= -1

        return True

    def check_game_over(self):
        # Rows and columns
        for i in range(3):
            s_row = int(self.board[i, :].sum())
            if abs(s_row) == 3:
                self.winner = self.current_player
                self.game_over = True
                return
            s_col = int(self.board[:, i].sum())
            if abs(s_col) == 3:
                self.winner = self.current_player
                self.game_over = True
                return

        # Diagonals
        diag1 = int(np.sum(np.diag(self.board)))
        if abs(diag1) == 3:
            self.winner = self.current_player
            self.game_over = True
            return
        diag2 = int(np.sum(np.diag(np.fliplr(self.board))))
        if abs(diag2) == 3:
            self.winner = self.current_player
            self.game_over = True
            return

        # Draw
        if np.count_nonzero(self.board == 0) == 0:
            self.game_over = True
            self.winner = 0


class MENACEAgent:
    def __init__(self, player_id: int):
        self.player_id = player_id
        self.beads = np.ones((19683, 9), dtype=np.int64)
        self.legal_moves_mask = np.ones((19683, 9), dtype=bool)
        self.game_history = []

    def _canonical_key(self, board: np.ndarray) -> int:
        canonical_board = board * self.player_id
        return encode_board_base3(canonical_board)

    def select_move(self, game: TicTacToe) -> Optional[Tuple[int, int]]:
        legal_moves = game.get_legal_moves()
        if not legal_moves:
            return None

        legal_indices = [r * 3 + c for (r, c) in legal_moves]
        state_key = self._canonical_key(game.board)

        beads_for_state = self.beads[state_key, :]
        beads_for_legal_moves = beads_for_state[legal_indices]

        total_beads = beads_for_legal_moves.sum()
        if total_beads <= 0:
            choice_index = np.random.choice(len(legal_indices))
            move_idx = legal_indices[choice_index]
        else:
            probs = beads_for_legal_moves / total_beads
            choice_index = np.random.choice(len(legal_indices), p=probs)
            move_idx = legal_indices[choice_index]

        row, col = divmod(move_idx, 3)
        self.game_history.append((state_key, move_idx))
        return (row, col)

    def update_beads(self, reward: int):
        for state_key, move_idx in self.game_history:
            if reward == -1:
                self.beads[state_key, move_idx] = max(
                    0, self.beads[state_key, move_idx] - 2
                )
            elif reward == 0:
                self.beads[state_key, move_idx] += 1
            elif reward == 1:
                self.beads[state_key, move_idx] += 4
        self.game_history.clear()


def load_agent_beads(agent: MENACEAgent, filename: str) -> bool:
    if not os.path.exists(filename):
        return False
    try:
        with open(filename, "rb") as f:
            agent.beads = np.load(f)
        return True
    except Exception as e:
        print(f"Error loading agent: {e}")
        return False


def save_agent_beads(agent: MENACEAgent, filename: str):
    with open(filename, "wb") as f:
        np.save(f, agent.beads)


class TicTacToeUI:
    """Pygame UI for visualization."""

    def __init__(self, human_player: int, width=600, height=700):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))

        self.human_player = human_player
        self.ai_player = -human_player

        human_symbol = "X" if human_player == 1 else "O"
        ai_symbol = "O" if human_player == 1 else "X"
        pygame.display.set_caption(
            f"MENACE Tic-Tac-Toe - Human ({human_symbol}) vs AI ({ai_symbol})"
        )

        self.cell_size = 150
        self.board_offset_x = (width - 3 * self.cell_size) // 2
        self.board_offset_y = 100

        self.font = pygame.font.Font(None, 72)
        self.info_font = pygame.font.Font(None, 36)

        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.BLUE = (0, 100, 200)
        self.RED = (200, 0, 0)
        self.GRAY = (128, 128, 128)

    def draw_board(self, game: TicTacToe, waiting_for_robot: bool = False):
        self.screen.fill(self.WHITE)

        # Draw grid lines
        for i in range(4):
            x = self.board_offset_x + i * self.cell_size
            pygame.draw.line(
                self.screen,
                self.BLACK,
                (x, self.board_offset_y),
                (x, self.board_offset_y + 3 * self.cell_size),
                3,
            )
            y = self.board_offset_y + i * self.cell_size
            pygame.draw.line(
                self.screen,
                self.BLACK,
                (self.board_offset_x, y),
                (self.board_offset_x + 3 * self.cell_size, y),
                3,
            )

        # Draw X's and O's
        for row in range(3):
            for col in range(3):
                cell_x = self.board_offset_x + col * self.cell_size
                cell_y = self.board_offset_y + row * self.cell_size
                center_x = cell_x + self.cell_size // 2
                center_y = cell_y + self.cell_size // 2

                if game.board[row, col] == 1:
                    text = self.font.render("X", True, self.BLUE)
                    text_rect = text.get_rect(center=(center_x, center_y))
                    self.screen.blit(text, text_rect)
                elif game.board[row, col] == -1:
                    text = self.font.render("O", True, self.RED)
                    text_rect = text.get_rect(center=(center_x, center_y))
                    self.screen.blit(text, text_rect)

        # Status text
        if game.game_over:
            if game.winner == self.human_player:
                info_text = "You Win!"
            elif game.winner == self.ai_player:
                info_text = "AI Wins!"
            else:
                info_text = "Draw!"
        else:
            if waiting_for_robot:
                ai_symbol = "X" if self.ai_player == 1 else "O"
                info_text = f"AI ({ai_symbol}) is drawing..."
            elif game.current_player == self.human_player:
                human_symbol = "X" if self.human_player == 1 else "O"
                info_text = f"Your turn ({human_symbol}) - Draw on the board"
            else:
                ai_symbol = "X" if self.ai_player == 1 else "O"
                info_text = f"AI ({ai_symbol}) is thinking..."

        text_surface = self.info_font.render(info_text, True, self.BLACK)
        text_rect = text_surface.get_rect(center=(self.width // 2, 50))
        self.screen.blit(text_surface, text_rect)

        # Instructions
        human_symbol = "X" if self.human_player == 1 else "O"
        ai_symbol = "X" if self.ai_player == 1 else "O"
        inst_text = (
            f"You are {human_symbol}, AI is {ai_symbol}. Space for next game, Q to quit"
        )
        inst_surface = self.info_font.render(inst_text, True, self.GRAY)
        inst_rect = inst_surface.get_rect(center=(self.width // 2, self.height - 50))
        self.screen.blit(inst_surface, inst_rect)

        pygame.display.flip()


class TicTacToeNode(Node):
    """
    ROS2 Node for Tic-Tac-Toe game orchestration.
    Coordinates between perception, manipulation, and the MENACE AI.
    """

    def __init__(self):
        super().__init__("tictactoe_node")

        # Declare parameters
        package_dir = get_package_share_directory("brain")
        self.declare_parameter("player", "x")
        self.declare_parameter(
            "agent_x_file", os.path.join(package_dir, "models", "menace_agent_x.npy")
        )
        self.declare_parameter(
            "agent_o_file", os.path.join(package_dir, "models", "menace_agent_o.npy")
        )

        # Get parameters
        player_str = self.get_parameter("player").value
        agent_x_file = self.get_parameter("agent_x_file").value
        agent_o_file = self.get_parameter("agent_o_file").value

        # Game setup
        self.human_player = 1 if player_str.lower() == "x" else -1
        self.ai_player = -self.human_player

        # Load AI agent
        if self.ai_player == 1:
            self.ai_agent = MENACEAgent(1)
            ai_file = agent_x_file
            self.ai_symbol = "X"
        else:
            self.ai_agent = MENACEAgent(-1)
            ai_file = agent_o_file
            self.ai_symbol = "O"

        if load_agent_beads(self.ai_agent, ai_file):
            self.get_logger().info(f"Loaded AI agent ({self.ai_symbol}) from {ai_file}")
        else:
            self.get_logger().warn(f"Failed to load {ai_file}. Using untrained agent.")

        self.game = TicTacToe()
        self.human_symbol = "X" if self.human_player == 1 else "O"

        # Statistics
        self.games_played = 0
        self.human_wins = 0
        self.ai_wins = 0
        self.draws = 0

        # Publishers
        self.game_state_pub = self.create_publisher(Int32MultiArray, "game_state", 10)
        self.move_request_pub = self.create_publisher(Int32, "robot_move_request", 10)
        self.game_status_pub = self.create_publisher(String, "game_status", 10)

        # Subscribers
        self.shutdown_sub = self.create_subscription(
            Bool, "/kb/shutdown", self.shutdown_callback, 10
        )
        self.shutdown_requested = False

        self.toggle_log_sub = self.create_subscription(
            Bool, "/kb/toggle_log", self.toggle_log_callback, 10
        )
        self.toggle_log = True

        self.grid_poses_sub = self.create_subscription(
            GridPose, "perception/cell_poses", self.grid_poses_callback, 10
        )

        # Default grid poses
        self.grid_poses = [
            Pose2D(x=0.55, y=0.03, theta=78.87),
            Pose2D(x=0.54, y=0.08, theta=78.87),
            Pose2D(x=0.53, y=0.14, theta=78.87),
            Pose2D(x=0.61, y=0.04, theta=78.87),
            Pose2D(x=0.60, y=0.10, theta=78.87),
            Pose2D(x=0.59, y=0.16, theta=78.87),
            Pose2D(x=0.67, y=0.05, theta=78.87),
            Pose2D(x=0.66, y=0.11, theta=78.87),
            Pose2D(x=0.65, y=0.17, theta=78.87),
        ]

        # Vision-based move detection
        self.UPDATE_FREQUENCY = 6.0  # Hz
        self.CONFIRMATION_TIME = 3.0  # seconds
        self.WINDOW_SIZE = int(self.UPDATE_FREQUENCY * self.CONFIRMATION_TIME)
        self.CONFIRMATION_THRESHOLD = 0.8
        self.cell_observations = [[] for _ in range(9)]

        self.draw_action_client = ActionClient(
            self, DrawShape, "manipulation/draw_shape"
        )
        self.waiting_for_robot = False
        self._pending_move = None
        self._constraint = (
            "ORIEN" if os.environ.get("ROS_DISTRO", "").lower() == "humble" else "NONE"
        )

        # Pygame UI
        self.ui = TicTacToeUI(self.human_player)
        self.clock = pygame.time.Clock()

        # Timer for pygame event processing
        self.timer = self.create_timer(0.016, self.process_ui)  # ~60 FPS

        self.get_logger().info(
            f"TicTacToe game started - Human ({self.human_symbol}) vs AI ({self.ai_symbol})"
        )
        self.get_logger().info("Waiting for human to draw their move on the board...")

        # If AI goes first, make its first move
        if self.ai_player == 1:
            self.make_ai_move()

        self.publish_game_state()

    def publish_game_state(self):
        """Publish current board state to other nodes."""
        msg = Int32MultiArray()
        msg.data = self.game.board.flatten().tolist()
        self.game_state_pub.publish(msg)

    def publish_game_status(self, status: str):
        """Publish game status message."""
        msg = String()
        msg.data = status
        self.game_status_pub.publish(msg)

    def make_ai_move(self):
        """Execute AI move and handle robot/perception integration."""
        ai_move = self.ai_agent.select_move(self.game)
        if ai_move:
            row, col = ai_move

            if self.toggle_log:
                self.get_logger().info(
                    f"AI ({self.ai_symbol}) selected move at row={row}, col={col}"
                )

            self.send_draw_shape_goal(row, col, self.ai_symbol)

    def send_draw_shape_goal(self, row: int, col: int, shape: str):
        """Send action goal to manipulation node to draw shape."""
        if self.grid_poses is None or len(self.grid_poses) < 9:
            self.get_logger().error("Grid poses not available!")
            return

        # Block game interactions
        self.waiting_for_robot = True

        # Wait for action server
        if not self.draw_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Draw shape action server not available!")
            self.waiting_for_robot = False
            return

        # Create goal
        self._pending_move = (row, col)
        cell_number = row * 3 + col
        goal_msg = DrawShape.Goal()
        goal_msg.pose = self.grid_poses[cell_number]
        goal_msg.shape = shape
        goal_msg.constraints_identifier = self._constraint

        if self.toggle_log:
            self.get_logger().info(f"Sending goal: Draw {shape} at cell {cell_number}")

        # Send goal with callbacks
        self._send_goal_future = self.draw_action_client.send_goal_async(
            goal_msg, feedback_callback=self.draw_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.draw_goal_response_callback)

    def draw_goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server!")
            self.waiting_for_robot = False
            return

        if self.toggle_log:
            self.get_logger().info("Goal accepted, waiting for result...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.draw_result_callback)

    def draw_feedback_callback(self, feedback_msg):
        """Handle feedback from action server."""
        feedback = feedback_msg.feedback
        if self.toggle_log:
            self.get_logger().info(
                f"Drawing progress: {feedback.status} - {feedback.progress:.1%}"
            )

    def draw_result_callback(self, future):
        """Handle action completion."""
        result = future.result().result
        self.waiting_for_robot = False

        if result.success and self._pending_move:
            if self.toggle_log:
                self.get_logger().info(f"Drawing completed: {result.message}")
            row, col = self._pending_move
            self.game.make_move(row, col)
            self.publish_game_state()
            self.check_game_end()

            # If game not over and it's human's turn, wait for their move
            if (
                not self.game.game_over
                and self.game.current_player == self.human_player
            ):
                self.get_logger().info("Waiting for human move...")
        elif self._pending_move:
            self.get_logger().error(f"Drawing failed: {result.message}. Retrying...")
            # Retry the move
            row, col = self._pending_move
            self.send_draw_shape_goal(row, col, self.ai_symbol)

    def check_game_end(self):
        """Check if game ended and update statistics."""
        if self.game.game_over:
            if self.game.winner == self.human_player:
                self.ai_agent.update_beads(-1)  # AI loses
                self.human_wins += 1
                self.publish_game_status(
                    f"Human wins! H:{self.human_wins} A:{self.ai_wins} D:{self.draws}"
                )
                self.get_logger().info(
                    f"Game {self.games_played + 1}: Human ({self.human_symbol}) wins. "
                    f"H:{self.human_wins} A:{self.ai_wins} D:{self.draws}"
                )
            elif self.game.winner == self.ai_player:
                self.ai_agent.update_beads(1)  # AI wins
                self.ai_wins += 1
                self.publish_game_status(
                    f"AI wins! H:{self.human_wins} A:{self.ai_wins} D:{self.draws}"
                )
                self.get_logger().info(
                    f"Game {self.games_played + 1}: AI ({self.ai_symbol}) wins. "
                    f"H:{self.human_wins} A:{self.ai_wins} D:{self.draws}"
                )
            else:
                self.ai_agent.update_beads(0)  # Draw
                self.draws += 1
                self.publish_game_status(
                    f"Draw! H:{self.human_wins} A:{self.ai_wins} D:{self.draws}"
                )
                self.get_logger().info(
                    f"Game {self.games_played + 1}: Draw. "
                    f"H:{self.human_wins} A:{self.ai_wins} D:{self.draws}"
                )

            self.games_played += 1

            # Save agent after each game
            if self.ai_player == 1:
                filename = self.get_parameter("agent_x_file").value
            else:
                filename = self.get_parameter("agent_o_file").value

            try:
                save_agent_beads(self.ai_agent, filename)
            except Exception as e:
                self.get_logger().warn(f"Failed to save agent: {e}")

    def reset_game(self):
        """Reset the game for a new round."""
        self.game.reset()
        self.cell_observations = [[] for _ in range(9)]
        self.publish_game_state()
        self.publish_game_status("New game started")
        self.get_logger().info("Game reset")

        # If AI goes first, make its move
        if self.ai_player == 1 and not self.game.game_over:
            self.make_ai_move()
        else:
            self.get_logger().info("Waiting for human move...")

    def grid_poses_callback(self, msg):
        """
        Receives board state from perception node and detects new human moves.
        """
        self.grid_poses = msg.poses

        # Don't process moves if waiting for robot or game is over
        if self.waiting_for_robot or self.game.game_over:
            return

        # Only process if it's human's turn
        if self.game.current_player != self.human_player:
            return

        new_colors = list(msg.colors)

        # Check for new human moves
        for i in range(9):

            # Skip if cell is already occupied in game
            row, col = divmod(i, 3)
            if self.game.board[row, col] != 0:
                self.cell_observations[i].clear()
                continue

            self.cell_observations[i].append(new_colors[i])
            if len(self.cell_observations[i]) > self.WINDOW_SIZE:
                self.cell_observations[i].pop(0)

            # Check if we have enough observations
            if len(self.cell_observations[i]) >= self.WINDOW_SIZE:
                human_count = self.cell_observations[i].count(self.human_player)
                confidence = human_count / len(self.cell_observations[i])

                if confidence >= self.CONFIRMATION_THRESHOLD:
                    if self.toggle_log:
                        self.get_logger().info(
                            f"Detected stable human move at ({row}, {col}) with confidence {confidence:.1f}"
                        )

                    # Register the move
                    self.game.make_move(row, col)
                    self.publish_game_state()
                    self.cell_observations = [[] for _ in range(9)]
                    self.check_game_end()

                    # If game not over, AI makes its move
                    if (
                        not self.game.game_over
                        and self.game.current_player == self.ai_player
                    ):
                        self.make_ai_move()
                    break

    def process_ui(self):
        """Process pygame events (called by timer)."""

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.get_logger().info("UI closed, shutting down node")
                pygame.quit()
                self.shutdown_requested = True
                return

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.reset_game()
                elif event.key == pygame.K_q:
                    self.get_logger().info("Quit requested, shutting down node")
                    pygame.quit()
                    self.shutdown_requested = True
                    return

        self.ui.draw_board(self.game, self.waiting_for_robot)

        # Show overlay if waiting for robot
        if self.waiting_for_robot:
            overlay = pygame.Surface((self.ui.width, self.ui.height))
            overlay.set_alpha(128)
            overlay.fill((255, 255, 255))
            self.ui.screen.blit(overlay, (0, 0))

            waiting_text = self.ui.info_font.render(
                "Robot is drawing...", True, (200, 0, 0)
            )
            waiting_rect = waiting_text.get_rect(
                center=(self.ui.width // 2, self.ui.height // 2)
            )
            self.ui.screen.blit(waiting_text, waiting_rect)

            pygame.display.flip()

        self.clock.tick(60)

    def cleanup(self):
        """Cleanup on node shutdown."""
        self.get_logger().info(f"\nFinal stats after {self.games_played} games:")
        self.get_logger().info(f"Human ({self.human_symbol}) wins: {self.human_wins}")
        self.get_logger().info(f"AI ({self.ai_symbol}) wins: {self.ai_wins}")
        self.get_logger().info(f"Draws: {self.draws}")
        pygame.quit()

    def shutdown_callback(self, msg):
        """Handle shutdown request from keyboard node."""
        self.get_logger().info("Shutdown requested via keyboard node")
        self.cleanup()
        self.shutdown_requested = True

    def toggle_log_callback(self, msg):
        """Handle toggle log request from keyboard node."""
        self.toggle_log = not self.toggle_log
        status = "enabled" if self.toggle_log else "disabled"
        self.get_logger().info(f"Logging {status} via keyboard node")


def main(args=None):
    rclpy.init(args=args)
    node = TicTacToeNode()

    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
