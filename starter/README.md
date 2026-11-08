# EC-3161 Mini-Project Starter Code

One folder per problem (P01-P23). Each starter file runs on its own (`python <file>.py`) and prints a
self-test. Starters provide the *environment* (problem generator, rules, simulator, data); every
algorithm you are asked to compare is yours to write. Python 3.10+, standard library only
(NumPy/Matplotlib/PyTorch allowed in your own code; P11 needs PyTorch or NumPy for the network).

| Folder | Provides |
|---|---|
| P01_route | seeded road-network generator, standard query |
| P02_puzzle15 | 15-puzzle state, successors, scrambler, Manhattan heuristic |
| P03_delivery | multi-package delivery world + Problem class |
| P04_memory_bounded | maze generator, BFS ground truth, memory meter |
| P05_sokoban | level parser, push rules, 3 test levels (BFS-verified solvable) |
| P06_tsp_local | fixed 50-city instance, tour utilities |
| P07_coverage | base-station coverage scenario, objective, greedy baseline |
| P08_tsp_exact | random instances, brute-force checker |
| P09_connect4 | Connect Four rules, random agent, game loop |
| P10_othello | Othello rules (perft-verified), random agent, game loop |
| P11_alphazero_ttt | tic-tac-toe API + exact minimax oracle |
| P12_wumpus | Wumpus World simulator (R&N rules), random baseline |
| P13_minesweeper | Minesweeper engine (safe first click), random baseline |
| P14_sat | random 3-SAT, DIMACS I/O, checker, Sudoku-to-CNF encoder |
| P15_miniprolog | tiny-Prolog parser + fictional family knowledge base |
| P16_timetabling | exam timetabling generator, checker, soft penalty |
| P17_sudoku_csp | Sudoku CSP helpers, 10 unique-solution puzzles |
| P18_frequency | channel-assignment instance generator, violation checker |
| P19_jobshop | job-shop instances, lower bound, schedule checker |
| P20_rostering | TA rostering instance, feasibility checker, cost |
| P21_csp_library | zebra puzzle and N-queens as (variables, domains, constraints) |
| P22_strips_planner | STRIPS actions, blocks/logistics generators, BFS baseline, validator |
| P23_graphplan_pop | cake, spare tire, air cargo domains; layered-plan validator |
