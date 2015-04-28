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
		
		private Graph graph;
		private Bounds bounds;
		private List<Node> open = new List<Node>();
		private List<Node> closed = new List<Node>();
		private Node startNode;
		private Node destinationNode;
		private Node nextNodeInPath;
		
		private float dist_down;
		
		public Astar() {
			
		}
		
		public IEnumerator SearchForPath () {
			if (graph == null) yield break;
			StartSearch();
			yield break;
		}
		public IEnumerator SearchForPath(Vector3 start, Vector3 end) {
			if (graph == null) yield break;
			origin = start;
			destination = end;
			StartSearch();
			yield break;
		}

		public Vector3 PathDirection (Vector3 myLocation) {
			// obstacle avoidance
			Vector3 direction = Vector3.zero;
			
			if (dist_down < 1f) direction += Vector3.up * graph.spacing;
			
			// path direction
			if (!nextNodeInPath) return Vector3.zero;
			origin = myLocation;
			// advance nextNodeInPath if we're close enough to it
			if (nextNodeInPath.state != Node.State.destination)
				if (Vector3.Distance(myLocation, nextNodeInPath.position + direction) < graph.spacing)
					nextNodeInPath = nextNodeInPath.child;
			
			direction += (nextNodeInPath.position - myLocation);
			
			return direction;
		}

		public void Proximity (Vector3 from, Vector3 to, bool obstructed) {
			
			// update graph
			Vector3 mark = from;
			float distance = Vector3.Distance(from, to);
			Vector3 direction = (to-from).normalized;
			float stepSize = graph.spacing;
			if (stepSize <= 0f) stepSize = 1f;
			// Mark nodes in zone "II" as walkable
			// i.e. inside this method n.type cannot go from obstructed to anything else
			/*
			for (float step = 0f; step < distance; step += stepSize) {
				mark = Vector3.Lerp(from, to, step/distance);
				Node n = graph.FindNearestNode(mark);
				n.type = Node.Type.walkable;
				
			}*/
			if (obstructed) {
				Node n = graph.FindNearestNode(to);
				n.type = Node.Type.obstructed;
				if (NodeInPath(n)) StartSearch();
			}
			

			if (Vector3.Angle(direction, Vector3.down) < 30f) {
				dist_down = distance;
			}
			
		}
		
		public Bounds searchBounds {
			get { return bounds; }
			set {
				bounds = value;
				graph = new CubeGraph(bounds, 0.5f);
				graph.RevealObstacles();
			}
		}

		public Vector3 origin {get; set;}
		public Vector3 destination {get; set;}
		public bool pathFound {get {return (bool)nextNodeInPath;} }
		public Space spaceRelativeTo { get { return Space.World; } }
	
		public void DrawGizmos() {
			graph.DrawGizmos();
		}
		
		public void DrawDebugInfo() {
			graph.DrawNodes();
			
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
			if (graph == null) {	
				return;
			}
			BuildPath();
		}
		
		void BuildPath() {
			bool success = false;
			
			startNode = graph.FindNearestUnobstructedNode(origin);
			startNode.state = Node.State.start;
			destinationNode = graph.FindNearestUnobstructedNode(destination);
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
				if (++i > graph.nodeCount) {
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
				if (current == n) return true;
				if (!current.parent) return false;
				current = current.parent;
				if (++i > graph.nodeCount) {
					Debug.LogError("A*: start node not found!");
					return false;
				}
			}
			return false;
		}
		

	}
	
	
	
}

