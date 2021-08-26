

Estimated size of a single from = ~16 bytes.
Priority queue will need another 4 byte per from (very worst case).
Path ID or visitTime will take 4 bytes.
Max size of a map Nmax = 2000
Grid size = sizeof(Node) + sizeof(NodeID) + 4 = 24 * 2000 * 2000 ~ 96Mb.



Input:

```
4 20 10
....
....
....
....
7 7
1
1 1 4 4  (1)
1
1 4 4 1  (2)
1
4 4 1 1  (3)
0
4
1 2 4 4  (4)
2 2 3 3  (5) - Problem here
2 1 4 4  (6)
2 2 4 4  (7)
0
0
```

```
wrong answer Rover 1 can't finish order 5 at iteration 6 because he does not reach destination yet

Interactor exit code : 1
```

Fixed.


Testing task08:

```
Done in 90745ms.
Managed to process only 6224 steps out of 100000
```

Removed unnecessary tracing:

```
Done in 76210ms.
Managed to process only 23526 steps out of 100000 (23%)
```

Removed heuristics:

```
Done in 71422ms.
Managed to process only 24942 steps out of 100000 (24%)
```

Removed extra pathfinding:

```
Done in 49061ms.
Managed to process only 36354 steps out of 100000 (36%)
```

# Tuning revenue at 08 #

Initial revenue and losses:

```
Done in 51737ms.
Revenue=12747388. Loss=25679232
Managed to process only 34385 steps out of 100000 (34%)
```

Multiplied necessary robots by 1.5:

```
Done in 55054ms.
Revenue=16436670. Loss=19779850
Managed to process only 32069 steps out of 100000 (32%)
```

Multiplied necessary robots by 1.9:

```
Done in 56429ms.
Revenue=19581027. Loss=16181967
Managed to process only 31335 steps out of 100000 (31%)
```

We've managed to 

# Reassigning tasks on fly #

```
Generated island with 16 elements
Avg order delay = 60. Avg delivery time = 8 steps. Need 1 robots
Max revenue = 140, expected revenue = 74
1
1 4
Assigning task 0 to robot 0
Order 0 is closed. Revenue =10, loss=10
LLLTRDDRRDPSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
Assigning task 1 to robot 0
Order 1 is closed. Revenue =10, loss=10
UUUTLDDLLDPSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
Assigning task 2 to robot 0
Order 2 is closed. Revenue =10, loss=10
RRRTULLUULPSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
Assigning task 3 to robot 0
No free candidates for task 4
Order 3 is closed. Revenue =13, loss=7
RTDDRDRPSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
Assigning task 4 to robot 0
No free candidates for task 5
Order 4 is closed. Revenue =0, loss=67
ULULTRDPSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
Assigning task 5 to robot 0
No free candidates for task 6
Order 5 is closed. Revenue =0, loss=129
LULTRDRRDPSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
Done in 26ms.
Revenue=33. Loss=233
```

```
Generated island with 16 elements
Avg order delay = 60. Avg delivery time = 8 steps. Need 1 robots
Max revenue = 140, expected revenue = 74
1
1 4
Assigning task 0 to robot 0
Order 0 is closed. Revenue =10, loss=10
LLLTRDDRRDPSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
Assigning task 1 to robot 0
Order 1 is closed. Revenue =10, loss=10
UUUTLDDLLDPSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
Assigning task 2 to robot 0
Order 2 is closed. Revenue =10, loss=10
RRRTULLUULPSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
Assigning task 3 to robot 0
No free candidates for task 4
Order 3 is closed. Revenue =13, loss=7
Assigning task 4 to robot 0
No free candidates for task 5
Order 4 is closed. Revenue =5, loss=15
Assigning task 5 to robot 0
No free candidates for task 6
Order 5 is closed. Revenue =0, loss=25
Assigning task 6 to robot 0
Order 6 is closed. Revenue =0, loss=35
RTDDRDRPULULTRDPLULTRDRRDPULULTRDRDPSSSSSSSSSSSSSSSSSSSSSSSS
SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
Done in 40ms.
Revenue=38. Loss=112
```