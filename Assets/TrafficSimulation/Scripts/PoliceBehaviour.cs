using Assets.TrafficSimulation.Scripts;
using System.Collections;
using System.Collections.Generic;
using TrafficSimulation;
using BehaviourAPI.Core;
using BehaviourAPI.StateMachines;
using BehaviourAPI.BehaviourTrees;
using UnityEngine;
using Assets.Scripts;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine.UIElements;
using static UnityEngine.GraphicsBuffer;
using BehaviourAPI.UnityToolkit.Demos;
using System;

enum PoliceStates
{
    PATROL,
    CHASE
}
namespace TrafficSimulation.Scripts
{
    public class PoliceBehaviour: MonoBehaviour
    {
        [SerializeField]
        private TrafficSystem trafficSystem;
        private PoliceStates state = PoliceStates.PATROL;
        [SerializeField]
        private Radius raycast;
        [SerializeField]
        private Radar radar;

        //Circulation
        private Node actualNode;
        private Node nextNode;

        //Car movement
        private Quaternion rotationAngle = Quaternion.identity;

        private WheelDrive_v2 wheelDrive;
        private float initMaxSpeed;

        private Coroutine currentCorutine = null;

        //Collisions
        private GameObject carDetected;
        private GameObject criminalDetected;
        private bool criminalCaught;
        private float safetyDist = 3f;

        private List<NodeInfo> path = new List<NodeInfo>();
        private Node nodeActualCriminal;
        private Stack<NodeInfo> currentPlan = new Stack<NodeInfo>();

        private bool stopped = false;

        public void Awake()
        {
            wheelDrive = GetComponent<WheelDrive_v2>();
            SetInitWaypoint();
        }

        public void Update()
        {
            if(state == PoliceStates.CHASE)
            {
                GameObject notify = radar.RadarActive();
                if (notify != null) notify.GetComponentInParent<CarBehaviour>().hearingHooter = true;
            }
        }

        #region PATROL
        public void Patrol()
        {
            state = PoliceStates.PATROL;

            initMaxSpeed = wheelDrive.maxSpeed;
            nextNode.waypoint = -1;
            

            if (currentCorutine != null)
            {
                StopCoroutine(currentCorutine);
                currentCorutine = null;
                SetInitWaypoint();
                criminalCaught = false;
            }

            currentCorutine = StartCoroutine(PatrolCoroutine());

            IEnumerator PatrolCoroutine()
            {
                while (true)
                {
                    //if (OnRefuelEntry()) yield break;
                    wheelDrive.maxSpeed = initMaxSpeed;
                    float acc = 0.5f;
                    float brake = 0.0f; //freno

                    if (CriminalDetected()) yield break;
                    if (IsInWaypoint())
                    {
                        actualNode = ChooseDirection();
                        //ChooseDirection();
                        
                    }
                    float hitDist;
                    if (CarDetected(out hitDist))
                    {
                        //Si el coche de delante va mas despacio
                        if (carDetected.GetComponent<WheelDrive_v2>().maxSpeed < wheelDrive.maxSpeed)
                        {
                            float ms = Mathf.Max(wheelDrive.GetSpeedMS(carDetected.GetComponent<WheelDrive_v2>().maxSpeed) - .5f, .1f);
                            wheelDrive.maxSpeed = wheelDrive.GetSpeedUnit(ms);
                        }
                        if (hitDist < safetyDist) //Choque inminente: FRENAR
                        {
                            acc = 0;
                            brake = 1;
                            wheelDrive.maxSpeed = Mathf.Max(wheelDrive.maxSpeed / 2f, wheelDrive.minSpeed);
                        }
                        else //Cerca: slowdown
                        {
                            acc = .25f;
                            brake = 0f;
                        }
                    }
                    if (raycast.PedestrianDetected()!=null)
                    {
                        acc = 0;
                        brake = 1;
                        wheelDrive.maxSpeed = Mathf.Max(wheelDrive.maxSpeed / 2f, wheelDrive.minSpeed);
                    }
                    if (WrongDirection(actualNode.transf.position))
                    {
                        //Slow down
                        acc = 0.1f;
                        wheelDrive.maxSpeed = Mathf.Max(wheelDrive.maxSpeed / 2f, wheelDrive.minSpeed);
                        //Comprobar q no hay nadie al girar
                        StartCoroutine(Rotate());
                        IEnumerator Rotate()
                        {
                            while (true)
                            {
                                transform.rotation = Quaternion.Slerp(transform.rotation, rotationAngle, Time.deltaTime * 1);
                                if (!WrongDirection(actualNode.transf.position)) //direccion corregida
                                {
                                    rotationAngle = Quaternion.identity;
                                    yield break;
                                }

                                yield return null;
                            }
                        }
                        if (raycast.CarInRadius()) //Comprobar q no hay nadie al girar
                        {
                            Debug.Log("Someone in intersection");
                            yield return new WaitUntil(() => { return !raycast.CarInRadius(); });
                            Debug.Log("Intersection free");
                        }
                    }
                    if (RedTrafficLight()) 
                    {
                        acc = 0.0f;
                        brake = 1.0f;
                        wheelDrive.maxSpeed = Mathf.Max(wheelDrive.maxSpeed / 2f, wheelDrive.minSpeed);
                    }

                    wheelDrive.Move(acc, brake);

                    yield return null;
                }
            }

        }
        #endregion

        #region CHASE
        public void Chase()
        {
            state = PoliceStates.CHASE;
            Debug.Log("CHASE");
            initMaxSpeed = wheelDrive.maxSpeed;
            nextNode.waypoint = -1;


            if (currentCorutine != null)
            {
                StopCoroutine(currentCorutine);
                currentCorutine = null;
                criminalCaught = false;
                nodeActualCriminal = criminalDetected.GetComponentInParent<CarBehaviour>().GetActualNode();
                Debug.Log("Node criminal: S" + nodeActualCriminal.segment + " W" + nodeActualCriminal.waypoint);
                //Nodos desde el actul hasta el del criminal
                NodeInfo init = new NodeInfo(actualNode.segment, actualNode.waypoint, actualNode.transf);
                if (nodeActualCriminal.segment != init.segment && nodeActualCriminal.waypoint != init.waypoint)
                {
                    NodeInfo goal = new NodeInfo(nodeActualCriminal.segment, nodeActualCriminal.waypoint, nodeActualCriminal.transf);

                    NodeInfo next = GetNextWaypoint(init, goal);
                    while (next != null)
                    {
                        path.Add(next);
                        next = GetNextWaypoint(init, goal);
                    }
                }
                else
                {
                    transform.LookAt(-transform.right);
                    criminalCaught = true;
                    return;
                }
            }

            currentCorutine = StartCoroutine(ChaseCorutine());
        
            IEnumerator ChaseCorutine()
            {
                
                while (true)
                {
                    //if (OnRefuelEntry()) yield break;
                    wheelDrive.maxSpeed = initMaxSpeed*3;
                    float acc = 1.0f;
                    float brake = 0.0f; //freno

                    if (criminalDetected != null && Vector3.Distance(transform.position, criminalDetected.transform.position) < 5.0f)
                    {
                        Debug.Log("PILLADO");
                        criminalCaught = true;
                    }
                    yield return new WaitForSeconds(1);
                    /*
                    if(raycast.CarInRadius())
                    {
                        Debug.Log(raycast.GetCar().ToString() + "||" + criminalDetected);
                        
                        if(raycast.GetCar() == criminalDetected)
                        {
                            Debug.Log("PILLADO");
                            criminalCaught = true;
                            yield break;
                        }
                                
                    }*/

                    //Guardar el camino que está siguiendo el otro coche para seguirle
                    if (criminalDetected.GetComponentInParent<CarBehaviour>().GetActualNode().transf != nodeActualCriminal.transf)
                    {
                        Node aux = criminalDetected.GetComponentInParent<CarBehaviour>().GetActualNode();
                        path.Add(new NodeInfo(aux.segment, aux.waypoint, aux.transf));
                        nodeActualCriminal.segment = aux.segment;
                        nodeActualCriminal.waypoint = aux.waypoint;
                        nodeActualCriminal.transf = aux.transf;
                        Debug.Log("New node: S" + aux.segment + " W" + aux.waypoint);
                    }
                    if (IsInWaypoint())
                    {
                        NodeInfo aux = path.First();
                        actualNode.segment = aux.segment;
                        actualNode.waypoint = aux.waypoint;
                        actualNode.transf = aux.transf;
                        path.RemoveAt(0);
                        //ChooseDirection();
                    }
                    
                    if (raycast.PedestrianDetected() != null)
                    {
                        acc = 0;
                        brake = 1;
                        wheelDrive.maxSpeed = Mathf.Max(wheelDrive.maxSpeed / 2f, wheelDrive.minSpeed);
                    }
                    if (WrongDirection(actualNode.transf.position))
                    {
                        //Slow down
                        acc = 0.5f;
                        wheelDrive.maxSpeed = Mathf.Max(wheelDrive.maxSpeed / 2f, wheelDrive.minSpeed);
                       
                        StartCoroutine(Rotate());
                        IEnumerator Rotate()
                        {
                            while (true)
                            {
                                transform.rotation = Quaternion.Slerp(transform.rotation, rotationAngle, Time.deltaTime * 1);
                                if (!WrongDirection(actualNode.transf.position)) //direccion corregida
                                {
                                    rotationAngle = Quaternion.identity;
                                    yield break;
                                }

                                yield return null;
                            }
                        }
                    }

                    wheelDrive.Move(acc, brake);
                    yield return null;
                }
            }

        }
        #endregion

        private void SetInitWaypoint()
        {
            //Find current target
            foreach (Segment segment in trafficSystem.segments)
            {
                if (segment.IsOnSegment(this.transform.position))
                {
                    actualNode.segment = segment.id;

                    //Find nearest waypoint to start within the segment
                    float minDist = float.MaxValue;
                    for (int j = 0; j < trafficSystem.segments[actualNode.segment].waypoints.Count; j++)
                    {
                        float d = Vector3.Distance(this.transform.position, trafficSystem.segments[actualNode.segment].waypoints[j].transform.position);

                        //Only take in front points
                        Vector3 lSpace = this.transform.InverseTransformPoint(trafficSystem.segments[actualNode.segment].waypoints[j].transform.position);
                        if (d < minDist && lSpace.z > 0)
                        {
                            minDist = d;
                            actualNode.waypoint = j;
                        }
                    }
                    break;
                }
            }
            actualNode.transf = trafficSystem.segments[actualNode.segment].waypoints[actualNode.waypoint].transform;
        }

        int GetNextSegmentId()
        {
            if (trafficSystem.segments[actualNode.segment].nextSegments.Count == 0)
                return 0;
            int c = UnityEngine.Random.Range(0, trafficSystem.segments[actualNode.segment].nextSegments.Count);
            return trafficSystem.segments[actualNode.segment].nextSegments[c].id;
        }

        private bool IsInWaypoint()
        {
            return Vector3.Distance(this.transform.position, actualNode.transf.position) < 1.2f;
        }

        private Node ChooseDirection()
        {
            //Debug.Log("Actual: Segmento " + actualNode.segment + ", Waypoint " + actualNode.waypoint);

            nextNode.waypoint = actualNode.waypoint + 1;
            nextNode.segment = actualNode.segment;

            if (nextNode.waypoint >= trafficSystem.segments[actualNode.segment].waypoints.Count)
            {
                nextNode.waypoint = 0;
                nextNode.segment = GetNextSegmentId();
            }
            nextNode.transf = trafficSystem.segments[nextNode.segment].waypoints[nextNode.waypoint].transform;
            //Debug.Log("Nueva direccion: Segmento " + nextNode.segment + ", Waypoint " + nextNode.waypoint);
            return nextNode;
        }

        private NodeInfo GetNextWaypoint(NodeInfo current, NodeInfo goal)
        {
            Debug.Log("Search path");
            if (currentPlan.Any())
            {
                return currentPlan.Pop();
            }
            else
            {
                var searchResult = SearchPath(current, goal);
                while (searchResult.parent != null)
                {
                    currentPlan.Push(searchResult.actual);
                    searchResult = searchResult.parent;
                }

                if (currentPlan.Any())
                {
                    return currentPlan.Pop();

                }
            }
            return null;
        }

        private Point SearchPath(NodeInfo start, NodeInfo goal)
        {
            List<Point> path = new List<Point>();

            Point n = new Point(start, goal);
            n.g = 0;
            path.Add(n);

            while (path.Any())
            {
                if (path[0].actual.segment == goal.segment && path[0].actual.waypoint == goal.waypoint)
                {
                    return path[0];
                }

                List<NodeInfo> hijos = path[0].actual.Neightbors(trafficSystem);
                //Debug.Log("Hijos nodo (S" + path[0].actual.SegmentID+", W" + path[0].actual.WaypointID+"): ");

                foreach (NodeInfo hijo in hijos)
                {
                    //Debug.Log(hijo.SegmentID + ", " + hijo.WaypointID);
                    Point h = new Point(hijo, goal);
                    h.parent = path[0];
                    if (h.parent != null)
                    {
                        h.g = h.parent.g + hijo.WalkCost;
                    }
                    h.funHeuristica();
                    path.Add(h);

                }

                //ordena lista: comprobar las f y ordenar de menor a mayor
                path.RemoveAt(0);
                path = path.OrderBy(n => n.f).ToList();
            }

            return null;
        }



        private bool RedTrafficLight()
        {
            Vector3 centreCar = new Vector3(transform.position.x, transform.position.y + 0.5f, transform.position.z);
            RaycastHit hit;
            if (Physics.Raycast(centreCar, transform.forward, out hit, 5))
            {
                if(hit.collider.CompareTag("TrafficLight")) return hit.collider.GetComponent<TrafficLights>().IsRedForCars();
            }
            return false;
        }
        private bool CarDetected(out float _hitDst)
        {
            Vector3 centreCar = new Vector3(transform.position.x, transform.position.y + 0.5f, transform.position.z);
            RaycastHit hit;
            if (Physics.Raycast(centreCar, transform.forward, out hit, 4f))
            {
                if (hit.collider.CompareTag("AutonomousVehicle"))
                {
                    carDetected = hit.collider.gameObject;
                    _hitDst = Vector3.Distance(transform.position, hit.collider.transform.position);
                    return true;
                }
            }
            _hitDst = -1f;
            return false;
        }

        #region Perceptions
        public bool CriminalDetected()
        {
            if(criminalDetected == null) criminalDetected = radar.RadarActive();
            if (criminalDetected != null && criminalDetected.GetComponent<WheelDrive_v2>().maxSpeed > initMaxSpeed) return true;
            else criminalDetected = null;
            return false;
        }

        public bool ChaseEnded()
        {
            return criminalCaught;
        }
        #endregion


        private bool WrongDirection(Vector3 target)
        {
            Vector3 direction = (target - transform.position).normalized;
            float angle = Mathf.Abs(1.0f - Vector3.Dot(direction, transform.forward));
            if (angle > 0.01f)
            {
                rotationAngle = Quaternion.LookRotation((target- transform.position).normalized);
                return true;
            }
            else
            {
                transform.LookAt(target);
            }
            return false;
        }

    }
}
