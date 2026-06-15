using System.Numerics;

namespace Roads.App.World;

/// <summary>
/// Generates a procedural rectangular grid road network for stress-testing the simulation.
/// Produces a cols x rows array of intersections connected by two-way road segments.
/// The returned lists are ready for direct consumption by <see cref="RoadGraph.LoadFromData"/>,
/// which rebuilds adjacency and turn matrices in bulk — do not call
/// <see cref="RoadGraph.AddEdge"/> per edge, as that rebuilds adjacency O(n) per call.
/// Call order: Generate → RoadGraph.LoadFromData.
/// </summary>
public static class GridNetworkGenerator
{
    /// <summary>
    /// Builds a <paramref name="cols"/> × <paramref name="rows"/> grid of intersections
    /// spaced <paramref name="spacing"/> meters apart, connected by two-way roads.
    /// Returns lists ready for <see cref="RoadGraph.LoadFromData(List{RoadNode}, List{RoadEdge})"/>.
    /// </summary>
    /// <remarks>
    /// Node index layout: row-major, index = r * cols + c, Position = (c * spacing, r * spacing).
    /// Each node's <see cref="RoadNode.EdgeStartIdx"/> and <see cref="RoadNode.EdgeCount"/> are
    /// left at zero; <c>LoadFromData</c> calls <c>RebuildAdjacency</c> to fill them.
    /// Two-way road connectivity is achieved by emitting two directed edges per connection:
    /// one forward (A→B) and one reverse (B→A), each with control points computed in its own
    /// direction to form a straight Bézier.
    /// For light realism, every 5th row and every 5th column use Arterial roads (LaneCount=2);
    /// all other roads are Residential (LaneCount=1).
    /// </remarks>
    /// <param name="cols">Number of columns of intersections (≥ 1).</param>
    /// <param name="rows">Number of rows of intersections (≥ 1).</param>
    /// <param name="spacing">Distance in meters between adjacent intersections (default 100 m).</param>
    /// <returns>
    /// A tuple of node and edge lists suitable for passing directly to
    /// <see cref="RoadGraph.LoadFromData"/>.
    /// </returns>
    public static (List<RoadNode> nodes, List<RoadEdge> edges) Generate(
        int cols, int rows, float spacing = 100f)
    {
        int nodeCount = cols * rows;
        // Each interior connection produces 2 directed edges (forward + reverse).
        // Horizontal connections: cols-1 per row, rows rows → (cols-1)*rows*2
        // Vertical connections:   rows-1 per col, cols cols → (rows-1)*cols*2
        int edgeCapacity = (cols - 1) * rows * 2 + (rows - 1) * cols * 2;

        var nodes = new List<RoadNode>(nodeCount);
        var edges = new List<RoadEdge>(edgeCapacity);

        // Build nodes in row-major order.
        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < cols; c++)
            {
                nodes.Add(new RoadNode
                {
                    Position = new Vector2(c * spacing, r * spacing),
                    EdgeStartIdx = 0,   // filled by RoadGraph.LoadFromData → RebuildAdjacency
                    EdgeCount = 0,      // filled by RoadGraph.LoadFromData → RebuildAdjacency
                    Flags = NodeFlags.None,
                    PointOfInterest = POIType.None,
                });
            }
        }

        // Connect each node only to its EAST (c+1) and SOUTH (r+1) neighbors.
        // This visits every undirected pair exactly once; each pair becomes two directed edges.
        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < cols; c++)
            {
                int idxA = r * cols + c;

                // --- East neighbor ---
                if (c + 1 < cols)
                {
                    int idxB = r * cols + (c + 1);
                    bool arterial = (c % 5 == 4) || (r % 5 == 4); // every 5th column or row
                    AddTwoWayEdge(edges, nodes, idxA, idxB, arterial);
                }

                // --- South neighbor ---
                if (r + 1 < rows)
                {
                    int idxB = (r + 1) * cols + c;
                    bool arterial = (c % 5 == 4) || (r % 5 == 4);
                    AddTwoWayEdge(edges, nodes, idxA, idxB, arterial);
                }
            }
        }

        return (nodes, edges);
    }

    /// <summary>
    /// Appends a forward edge (A→B) and a reverse edge (B→A) to <paramref name="edges"/>.
    /// Control points are placed at 1/3 and 2/3 of the straight line in each direction,
    /// mirroring <see cref="RoadGraph.AddEdge"/> geometry.
    /// </summary>
    private static void AddTwoWayEdge(
        List<RoadEdge> edges,
        List<RoadNode> nodes,
        int idxA, int idxB,
        bool arterial)
    {
        RoadType roadType = arterial ? RoadType.Arterial : RoadType.Residential;
        byte laneCount = arterial ? (byte)2 : (byte)1;
        float speedLimit = RoadTypeDefaults.GetDefaultSpeedLimit(roadType);

        Vector2 posA = nodes[idxA].Position;
        Vector2 posB = nodes[idxB].Position;

        // Forward A → B
        Vector2 diffAB = posB - posA;
        float length = diffAB.Length();
        edges.Add(new RoadEdge
        {
            FromNode = idxA,
            ToNode = idxB,
            Length = length,
            SpeedLimit = speedLimit,
            LaneCount = laneCount,
            RoadType = roadType,
            Flags = EdgeFlags.None,
            ControlPoint1 = posA + diffAB * (1f / 3f),
            ControlPoint2 = posA + diffAB * (2f / 3f),
        });

        // Reverse B → A
        Vector2 diffBA = posA - posB;
        edges.Add(new RoadEdge
        {
            FromNode = idxB,
            ToNode = idxA,
            Length = length,  // same arc length
            SpeedLimit = speedLimit,
            LaneCount = laneCount,
            RoadType = roadType,
            Flags = EdgeFlags.None,
            ControlPoint1 = posB + diffBA * (1f / 3f),
            ControlPoint2 = posB + diffBA * (2f / 3f),
        });
    }
}
