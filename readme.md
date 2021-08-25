

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