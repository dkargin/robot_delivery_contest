

/// Estimated size of a single from = ~16 bytes.
/// Priority queue will need another 4 byte per from (very worst case).
/// Path ID or visitTime will take 4 bytes.
/// Max size of a map Nmax = 2000
/// Grid size = sizeof(Node) + sizeof(NodeID) + 4 = 24 * 2000 * 2000 ~ 96Mb.
/// 
