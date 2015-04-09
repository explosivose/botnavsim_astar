//------------------------------------------------------------------------------
/*

A* Search Plugin for BotNavSim

Notes on this implementation:
Not optimized
Uses manhatten method
Square graph where every node is connected to 8 neighbors 
(excluding nodes on the edge of the square)


*/
//------------------------------------------------------------------------------
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Astar {

	/// <summary>
	/// Astar.
	/// </summary>
	public class Astar : INavigation {
		
		private SquareGraph graphData;
		private Bounds bounds;
		private List<Node> open = new List<Node>();
		private List<Node> closed = new List<Node>();
		private Node startNode;
		private Node destinationNode;
		private Node nextNodeInPath;
		
		public Astar() {
			
		}
		
		public IEnumerator SearchForPath () {
			if (graphData == null) yield break;
			StartSearch();
			yield break;
		}
		public IEnumerator SearchForPath(Vector3 start, Vector3 end) {
			if (graphData == null) yield break;
			origin = start;
			destination = end;
			StartSearch();
			yield break;
		}

		public Vector3 PathDirection (Vector3 myLocation) {
			if (!nextNodeInPath) return Vector3.zero;
			origin = myLocation;
			// advance nextNodeInPath if we're close enough to it
			if (nextNodeInPath.state != Node.State.destination)
				if (Vector3.Distance(myLocation, nextNodeInPath.position) < graphData.spacing * 0.6f)
					nextNodeInPath = nextNodeInPath.child;
			
			return (nextNodeInPath.position - myLocation);
		}

		public void Proximity (Vector3 from, Vector3 to, float FOV, bool obstructed) {
			Vector3 mark = from;
			float length = Vector3.Distance(from, to);
			Vector3 direction = (to-from).normalized;
			float step = graphData.spacing;
			if (step <= 0f) step = 1f;
			// Mark nodes in zone "II" as walkable
			// i.e. inside this method n.type cannot go from obstructed to anything else
			for (float dist = 0f; dist < length; dist += step) {
				mark = Vector3.Lerp(from, to, dist/length);
				Node n = graphData.NearestNode(mark);
				n.type = Node.Type.walkable;
				
			}
			if (obstructed) {
				Node n = graphData.NearestNode(to);
				n.type = Node.Type.obstructed;
				if (NodeInPath(n)) StartSearch();
				// for close poximity mark nodes in arc
				/*if (length < graphData.spacing * 5f) {
					for (float a = -FOV; a < FOV; a+=2f) {
						Quaternion rotation = Quaternion.Euler(new Vector3(0,a,0));
						Vector3 arc = rotation * direction;
						n = graphData.NearestNode(from + arc * length);
						n.type = Node.Type.obstructed;
					}
				}*/

			}
		}
		
		public Bounds searchBounds {
			get { return bounds; }
			set {
				bounds = value;
				graphData = new SquareGraph(bounds.min, bounds.max, 50);
			}
		}

		public Vector3 origin {get; set;}
		public Vector3 destination {get; set;}
		public bool pathFound {get {return (bool)nextNodeInPath;} }
		public Space spaceRelativeTo { get { return Space.World; } }
	
		public void DrawGizmos() {
			graphData.DrawGizmos();
			
		}
		
		public void DrawDebugInfo() {
			graphData.DrawDebug(origin.y);
		}
		
		// Private Methods
		
		/// <summary>
		/// Starts the A* search to destination.
		/// </summary>
		private void StartSearch() {

			// Forget previous search data
			startNode = null;
			nextNodeInPath = null;
			foreach(Node n in closed) {
				n.child = null;
				n.parent = null;
				n.destination = null;
				n.state = Node.State.regular;
			}
			closed.Clear();
			foreach(Node n in open) {
				n.child = null;
				n.parent = null;
				n.destination = null;
				n.state = Node.State.regular;
			}
			open.Clear();
			// do nothing if graphData not set.
			if (graphData == null) {	
				return;
			}
			BuildPath();
		}
		
		void BuildPath() {
			bool success = false;
			
			startNode = graphData.NearestUnobstructedNode(origin);
			startNode.state = Node.State.start;
			destinationNode = graphData.NearestUnobstructedNode(destination);
			destinationNode.state = Node.State.destination;
			
			open.Add(startNode);
			Node current;
			while ( open.Count > 0 ) {
				current = LowestFscoreInOpen();
				if (current.state == Node.State.destination) {
					ReconstructPath();
					success = true;
					break;
				}
				
				current.destination = destinationNode;
				
				open.Remove(current);
				closed.Add(current);
				if (current.state != Node.State.start)
					current.state = Node.State.closed;
				
				foreach(Node n in current.connected) {
					if (closed.Contains(n)) continue;
					if (n.type == Node.Type.obstructed) continue;
					
					n.destination = destinationNode;
					
					if (!open.Contains(n) || n.TentativeG(current) < n.G) {
						n.parent = current;
						if (!open.Contains(n)) {
							open.Add(n);
							if (n.state != Node.State.destination)
								n.state = Node.State.open;
						}
					}
				}
			}
			
			if (!success) {
				Debug.LogWarning("A*: Could not find path to destination.");
			}
		}
		
		void ReconstructPath() {
			Node current = destinationNode;
			int i = 0;
			while (current != startNode) {
				if (current.parent) {
					current.parent.child = current;
					current = current.parent;
					if (current.state == Node.State.closed)
						current.state = Node.State.path;
				}
				if (++i > graphData.graph.Length) {
					Debug.LogError("A*: Reconstruct Path Failed!");
					break;
				}
			}
			nextNodeInPath = current;
		}
		
		/// <summary>
		/// Gets the lowest f score in open list.
		/// </summary>
		/// <returns>The lowest f score in open list.</returns>
		private Node LowestFscoreInOpen() {
			Node lowest = open[0];
			foreach(Node n in open) {
				if (n.F < lowest.F) lowest = n;
			}
			return lowest;
		}
		
		/// <summary>
		/// Check if node n is featured in the current A* path.
		/// </summary>
		/// <returns><c>true</c>, if node is in path, <c>false</c> otherwise.</returns>
		/// <param name="n">N.</param>
		bool NodeInPath(Node n) {
			if (!nextNodeInPath) return false;
			Node current = destinationNode;
			int i = 0;
			while(current.state != Node.State.start) {
				if (current.index == n.index) return true;
				if (!current.parent) return false;
				current = current.parent;
				if (++i > graphData.graph.Length) {
					Debug.LogError("A*: start node not found!");
					return false;
				}
			}
			return false;
		}
		

	}
	
	
	
}

