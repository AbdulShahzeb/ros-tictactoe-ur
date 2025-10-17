#!/usr/bin/env python3
"""
Training script for MENACE agents.
Run this standalone to generate pkl files for the ROS2 node.

Usage:
    python3 train_menace.py --num-games 100000 --processes 8
"""

import numpy as np
import argparse
import os
import time
import multiprocessing as mp


def encode_board_base3(board_array: np.ndarray) -> int:
    """
    Converts a 3x3 board array (with values -1, 0, 1) to a base-3 integer.
    The values are mapped as: -1 -> 2, 0 -> 0, 1 -> 1.
    This creates a unique integer key for each board state.
    """
    board_flat = board_array.flatten()
    # Map -1 to 2, 0 to 0, 1 to 1 for base-3 representation
    mapped_board = np.where(board_flat == -1, 2, np.where(board_flat == 0, 0, 1))

    # Calculate base-3 integer
    base_3 = 0
    for i in range(9):
        base_3 += mapped_board[i] * (3**i)
    return base_3


def decode_board_base3(base3_int: int) -> np.ndarray:
    """
    Decodes a base-3 integer back to a 3x3 board array.
    """
    mapped_board = np.zeros(9, dtype=int)
    for i in range(9):
        mapped_board[i] = (base3_int // (3**i)) % 3
    # Map back to original values: 2 -> -1, 0 -> 0, 1 -> 1
    board_flat = np.where(mapped_board == 2, -1, np.where(mapped_board == 0, 0, 1))
    return board_flat.reshape((3, 3))


class TicTacToe:
    """
    Game Engine.
    """

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

    def get_legal_moves(self):
        return [(i, j) for i in range(3) for j in range(3) if self.board[i, j] == 0]

    def make_move(self, row: int, col: int) -> bool:
        if self.game_over or self.board[row, col] != 0:
            return False

        self.board[row, col] = self.current_player
        self.check_game_over()

        if not self.game_over:
            self.current_player *= -1  # Switch player

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
        self.player_id = player_id  # 1 for X, -1 for O
        self.beads = np.ones((19683, 9), dtype=np.int64)

        # We need a mask to "zero out" illegal moves for each state.
        self.legal_moves_mask = np.ones((19683, 9), dtype=bool)

        self.game_history = []  # list of (state_key, move_idx)

    def _canonical_key(self, board: np.ndarray) -> int:
        """
        Calculates the canonical state key from the agent's perspective.
        """
        canonical_board = board * self.player_id
        return encode_board_base3(canonical_board)

    def select_move(self, game: TicTacToe):
        legal_moves = game.get_legal_moves()
        if not legal_moves:
            return None

        legal_indices = [r * 3 + c for (r, c) in legal_moves]
        state_key = self._canonical_key(game.board)

        # Get beads for the canonical state and apply legal move mask
        beads_for_state = self.beads[state_key, :]
        beads_for_legal_moves = beads_for_state[legal_indices]

        total_beads = beads_for_legal_moves.sum()
        if total_beads <= 0:
            # If no beads are left, select a move uniformly from legal moves
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
        """
        reward: 1 win, 0 draw, -1 loss
        Updates the beads using the game history.
        """
        for state_key, move_idx in self.game_history:
            if reward == -1:  # loss: remove beads
                self.beads[state_key, move_idx] = max(
                    0, self.beads[state_key, move_idx] - 2
                )
            elif reward == 0:  # draw: small reward
                self.beads[state_key, move_idx] += 1
            elif reward == 1:  # win: stronger reward
                self.beads[state_key, move_idx] += 4
        self.game_history.clear()


def save_agent_beads(agent: MENACEAgent, filename: str):
    with open(filename, "wb") as f:
        np.save(f, agent.beads)


def run_batch_of_games(num_games: int):
    """
    Worker function to run a batch of games in a single process.
    Returns the accumulated bead updates.
    """
    agent_x = MENACEAgent(1)
    agent_o = MENACEAgent(-1)

    bead_updates_x = np.zeros((19683, 9), dtype=np.int64)
    bead_updates_o = np.zeros((19683, 9), dtype=np.int64)

    x_wins = 0
    o_wins = 0
    draws = 0

    for _ in range(num_games):
        game = TicTacToe()
        game_history_x = []
        game_history_o = []

        while not game.game_over:
            if game.current_player == 1:
                legal_moves = game.get_legal_moves()
                legal_indices = [r * 3 + c for (r, c) in legal_moves]
                state_key = agent_x._canonical_key(game.board)

                beads_for_state = agent_x.beads[state_key, :]
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
                game_history_x.append((state_key, move_idx))
                game.make_move(row, col)
            else:
                legal_moves = game.get_legal_moves()
                legal_indices = [r * 3 + c for (r, c) in legal_moves]
                state_key = agent_o._canonical_key(game.board)

                beads_for_state = agent_o.beads[state_key, :]
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
                game_history_o.append((state_key, move_idx))
                game.make_move(row, col)

        # Update bead counts in the local arrays
        if game.winner == 1:
            for state_key, move_idx in game_history_x:
                bead_updates_x[state_key, move_idx] += 4
            for state_key, move_idx in game_history_o:
                bead_updates_o[state_key, move_idx] -= 2
            x_wins += 1
        elif game.winner == -1:
            for state_key, move_idx in game_history_x:
                bead_updates_x[state_key, move_idx] -= 2
            for state_key, move_idx in game_history_o:
                bead_updates_o[state_key, move_idx] += 4
            o_wins += 1
        else:
            for state_key, move_idx in game_history_x:
                bead_updates_x[state_key, move_idx] += 1
            for state_key, move_idx in game_history_o:
                bead_updates_o[state_key, move_idx] += 1
            draws += 1

    # Return updates and stats for the batch
    return bead_updates_x, bead_updates_o, x_wins, o_wins, draws


def train_agents_vectorized(num_games: int, num_processes: int = mp.cpu_count()):
    """
    Trains agents using vectorized batches and multiprocessing.
    """
    print(f"Starting vectorized training with {num_processes} processes...")
    start_time = time.time()

    agent_x = MENACEAgent(1)
    agent_o = MENACEAgent(-1)

    batch_size = num_games // num_processes

    pool = mp.Pool(processes=num_processes)

    # Distribute the work across the processes
    results = pool.map(run_batch_of_games, [batch_size] * num_processes)

    pool.close()
    pool.join()

    # Aggregate results from all processes
    total_x_wins = 0
    total_o_wins = 0
    total_draws = 0

    final_bead_updates_x = np.zeros((19683, 9), dtype=np.int64)
    final_bead_updates_o = np.zeros((19683, 9), dtype=np.int64)

    for bead_updates_x, bead_updates_o, x_wins, o_wins, draws in results:
        final_bead_updates_x += bead_updates_x
        final_bead_updates_o += bead_updates_o
        total_x_wins += x_wins
        total_o_wins += o_wins
        total_draws += draws

    # Apply the final aggregated updates to the master agents
    agent_x.beads = np.maximum(0, agent_x.beads + final_bead_updates_x)
    agent_o.beads = np.maximum(0, agent_o.beads + final_bead_updates_o)

    end_time = time.time()
    print(f"Vectorized training complete in {end_time - start_time:.2f} seconds.")
    print(f"Total Games: {total_x_wins + total_o_wins + total_draws}")
    print(f"Final Stats: X:{total_x_wins} O:{total_o_wins} D:{total_draws}")

    return agent_x, agent_o


def main():
    parser = argparse.ArgumentParser(description="Train MENACE Tic-Tac-Toe Agents")
    parser.add_argument(
        "--num-games",
        type=int,
        default=100000,
        help="Number of self-play games to train (default: 100000)",
    )
    parser.add_argument(
        "--processes",
        type=int,
        default=int(mp.cpu_count() * 3 / 4),
        help="Number of processes to use for training.",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=".",
        help="Directory to save the agent pkl files (default: current directory)",
    )
    parser.add_argument(
        "--agent-x-file",
        type=str,
        default="menace_agent_x.npy",
        help="Filename for X agent (default: menace_agent_x.npy)",
    )
    parser.add_argument(
        "--agent-o-file",
        type=str,
        default="menace_agent_o.npy",
        help="Filename for O agent (default: menace_agent_o.npy)",
    )

    args = parser.parse_args()

    # Create output directory if it doesn't exist
    os.makedirs(args.output_dir, exist_ok=True)

    x_file = os.path.join(args.output_dir, args.agent_x_file)
    o_file = os.path.join(args.output_dir, args.agent_o_file)

    print(f"Training {args.num_games} self-play games...")
    agent_x, agent_o = train_agents_vectorized(args.num_games, args.processes)

    # Save both agents
    save_agent_beads(agent_x, x_file)
    save_agent_beads(agent_o, o_file)

    print(f"\nTraining complete! Saved agents:")
    print(f"  X agent: {x_file}")
    print(f"  O agent: {o_file}")
    print(f"\nYou can now use these files with the ROS2 tictactoe node.")


if __name__ == "__main__":
    main()
