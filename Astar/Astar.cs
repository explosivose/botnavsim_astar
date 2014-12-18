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
	/// Square graph.
	/// </summary>
	public class SquareGraph {
		private int X = 25;
		private int Y = 25;
		public float spacing = 1f;
		public Node[,] graph {get; set;}
		
		public SquareGraph(Vector3 min, Vector3 max, int nodes) {
			X = Y = nodes;
			spacing = (max.x - min.x)/(float)nodes;
			spacing = Mathf.Max(spacing, (max.z - min.z)/(float)nodes);
			graph = new Node[X,Y];
			for (int x = 0; x < X; x++) {
				for (int y = 0; y < Y; y++) {
					Vector3 position = new Vector3(x * spacing, 0, y * spacing);
					position += (min + Vector3.up*(max.y + min.y)/2f);
					Node n = new Node(position, null, this);
					graph[x,y] = n;
				}
			}
			
			ConnectNodes();
		}
		
		public void RevealObstacles() {
			// call every Node.CheckForObstacles()
			// note that Physics obstacleLayer should be shared with 
			// this plugin before this is implemented...
			throw new NotImplementedException();
		}
		
		void ConnectNodes() {
			for (int x = 0; x < X; x++) {
				for (int y = 0; y < Y; y++) {
					Node n = graph[x,y];
					if (y > 0) {
						n.connected.Add(graph[x,y-1]);
						if (x > 1)
							n.connected.Add(graph[x-1,y-1]);
						if (x < X-1)
							n.connected.Add(graph[x+1,y-1]);
					}
					if (x > 0) {
						n.connected.Add(graph[x-1,y]);
						if (y < Y-1)
							n.connected.Add(graph[x-1,y+1]);
					}
					if (x < X-1) {
						n.connected.Add(graph[x+1,y]);
						if (y < Y-1)
							n.connected.Add(graph[x+1,y+1]);
					}
					if (y < Y-1) {
						n.connected.Add(graph[x,y+1]);
					}
				}
			}
		}
		
		/// <summary>
		/// Returns the Node with position nearest to a specified world location.
		/// </summary>
		public Node NearestNode(Vector3 position) {
			Node nearestNode = graph[0,0];
			float d1 = Mathf.Infinity;
			for (int x = 0; x < X; x++) {
				for (int y = 0; y < Y; y++) {
					float d2 = Vector3.Distance(graph[x,y].position, position);
					if (d2 < d1) {
						nearestNode = graph[x,y];
						d1 = d2;
					}
				}
			}
			return nearestNode;
		}
		
		/// <summary>
		/// Returns the unobstructed Node with position nearest to a specified world location.
		/// </summary>
		public Node NearestUnobstructedNode(Vector3 position) {
			Node nearestNode = graph[0,0];
			float d1 = Mathf.Infinity;
			for (int x = 0; x < X; x++) {
				for (int y = 0; y < Y; y++) {
					if (graph[x,y].type != Node.Type.obstructed) {
						float d2 = Vector3.Distance(graph[x,y].position, position);
						if (d2 < d1) {
							nearestNode = graph[x,y];
							d1 = d2;
						}
					}
				}
			}
			return nearestNode;
		}
		
		public void DrawGizmos() {
			for (int x = 0; x < X; x++) {
				for (int y = 0; y < Y; y++) {
					graph[x,y].DrawGizmos();
					graph[x,y].DrawLines();
				}
			}
		}
		
		public void DrawDebug() {
			for (int x = 0; x < X; x++) {
				for (int y = 0; y < Y; y++) {
					graph[x,y].DrawLines();
				}
			}
		}
	}
	

	
	/// <summary>
	/// Node.
	/// </summary>
	public class Node {
		public enum State {
			regular,
			start,
			destination,
			path,
			open,
			closed
		}
		public enum Type {
			walkable,
			obstructed,
			unexplored
		}
		
		public SquareGraph graph;
		public List<Node> connected = new List<Node>();
		
		/// <summary>
		/// Set true if node state, type, parent of child is changed.
		/// </summary>
		public bool hasChanged;
		
		public Vector3 position {
			get; private set;
		}
		
		
		public float G { get; private set; }		// Astar G score
		public float H { get; private set; }		// Astar heuristic
		public float F { get { return G + H; } }	// Astar F score
		public int index { get; private set; } 		// unique index
		public Color color { get; private set; }	// debug color
		
		public Node parent {
			get { return _parent; }
			set {
				_parent = value;
				hasChanged = true;
				if (_parent) {
					G = _parent.G;
					G += Vector3.Distance(position, _parent.position);
				}
				else {
					G = 0f;
					type = type;
				}
			}
		}
		
		public Node child {
			get { return _child; }
			set {
				_child = value;
				hasChanged = true;
			}
		}
		
		public Node destination {
			get { return _destination; }
			set {
				_destination = value;
				if (_destination) {
					H = Mathf.Abs(position.x - _destination.position.x);
					H += Mathf.Abs(position.y - _destination.position.y);
					H += Math.Abs(position.z - _destination.position.z);
				}
				else {
					H = 0f;
				}
			}
		}
		
		public State state {
			get { return _state; }
			set {
				_state = value;
				hasChanged = true;
				switch(_state) {
				case State.regular:
					type = _type;
					break;
				case State.start:
					color = Color.Lerp(Color.clear, Color.yellow, 0.75f);
					break;
				case State.destination:
					color = Color.green;
					break;
				case State.path:
					color = Color.Lerp(Color.clear, Color.green, 0.75f);
					break;
				case State.closed:
					color = Color.Lerp(Color.clear, Color.cyan, 0.75f);
					break;
				case State.open:
					color = Color.Lerp(Color.clear, Color.magenta, 0.75f);
					break;
				default:
					color = Color.magenta;
					break; 
				}
			}
		}
		
		public Type type {
			get{ return _type; }
			set {
				_type = value;
				hasChanged = true;
				if (state != State.regular) return;
				switch(_type){
				case Type.obstructed:
					color = Color.Lerp(Color.clear, Color.red, 0.75f);
					break;
				case Type.walkable:
					color = Color.Lerp(Color.clear, Color.green, 0.25f);
					break;
				case Type.unexplored:
					color = Color.Lerp(Color.clear, Color.black, 0.25f);
					break;
				}
			}
		}
		
		
		private static int node_count;
		
		private State _state;
		private Type _type;
		private Node _parent;
		private Node _child;
		private Node _destination;
		
		public static implicit operator bool (Node n) {
			return n != null;
		}
		
		/// <summary>
		/// Initializes a new instance of the <see cref="Astar.Node"/> class.
		/// </summary>
		/// <param name="location">This node location.</param>
		/// <param name="nodePrefab">Node prefab used for runtime graph rendering. Set to null to ignore.</param>
		/// <param name="g">The graph that this node belongs to.</param>
		public Node(Vector3 location, Transform nodePrefab, SquareGraph g) {
			index = node_count++;
			position = location;
			state = State.regular;
			type = Type.unexplored;
			color = Color.white;
			graph = g;
		}
		
		/// <summary>
		/// Check for obstacles at node position
		/// (Physics.layer not implemented yet!)
		/// </summary>
		public void CheckForObstacles() {
			if (Physics.CheckSphere(position, graph.spacing-0.5f)) {
				type = Type.obstructed;
			}
			else {
				type = Type.walkable;
			}
		}
		
		public float TentativeG(Node potentialParent) {
			float tG = potentialParent.G;
			tG += Vector3.Distance(position, potentialParent.position);
			return tG;
		}
		
		// Draw debug information in worldspace (should be called by OnDrawGizmos() )
		public void DrawGizmos() {
			
			if (type == Type.obstructed) {
				Gizmos.color = Color.red;
				Gizmos.DrawWireCube(
					position,
					Vector3.one
					);
				return;
			}
			
			if (state == State.regular) return;
			
			Gizmos.color = color;
			Gizmos.DrawWireCube(
				position,
				Vector3.one * graph.spacing * 0.25f
				);
			
			if (parent) {
				Gizmos.DrawLine(position, parent.position);
			}
			else {
				foreach(Node n in connected) {
					Gizmos.DrawLine(position, n.position);
				}
			}
			
		}
		
		public void DrawLines() {
			if (type == Type.obstructed) {
				Draw.Instance.Cube(
					position, Vector3.one * 0.25f,
					Color.red);
				return;
			}
			
			if (state == State.regular) return;
			
			Draw.Instance.Line (
				position, position + Vector3.down,
				color);
			
			if (parent) {
				Draw.Instance.Line(
					position,parent.position,
					color);
			}
			else {
				foreach (Node n in connected) {
					Draw.Instance.Line(
						position, n.position, 
						color);
				}
			}
			
		}
	}
	
	
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
			
			return (nextNodeInPath.position - myLocation).normalized;
		}

		public void Proximity (Vector3 from, Vector3 to, bool obstructed) {
			Vector3 mark = from;
			float length = Vector3.Distance(from, to);
			float step = graphData.spacing;
			if (step <= 0f) step = 1f;
			// Mark nodes in zone "II" as walkable
			// Note that this implementation marks obstructed nodes permantently
			// i.e. inside this method n.type cannot go from obstructed to anything else
			for (float dist = 0f; dist < length; dist += step) {
				mark = Vector3.Lerp(from, to, dist/length);
				Node n = graphData.NearestNode(mark);
				if (n.type != Node.Type.obstructed) {
					n.type = Node.Type.walkable;
				}
			}
			if (obstructed) {
				Node n = graphData.NearestNode(to);
				n.type = Node.Type.obstructed;
				if (NodeInPath(n)) StartSearch();
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
			graphData.DrawDebug();
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

