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

enum CarStates
{
    CIRCULATE,
    REFUEL,
    STEP_ON
}
namespace TrafficSimulation.Scripts
{
    public class CarBehaviour : MonoBehaviour
    {
        private struct Node
        {
            public int segment;
            public int waypoint;
            public Transform transf;
        }

        [SerializeField]
        private TrafficSystem trafficSystem;
        [SerializeField]
        private CarStates state = CarStates.CIRCULATE;
        [SerializeField]
        private Radius raycast;

        [Header("Gasoline")]
        [SerializeField]
        protected int maxGasoline = 100;
        public int currentGasoline;
        public Transform FuelStation;

        private int gasolineDecreaser = 1000;
        private int repostTime = 10;

        //Circulation
        private Node actualNode;
        private Node nextNode;

        //Car movement
        private Quaternion rotationAngle = Quaternion.identity;

        private WheelDrive_v2 wheelDrive;
        private float initMaxSpeed;

        private Coroutine currentCorutine = null;

        //AStarPath
        private Stack<NodeInfo> currentPlan = new Stack<NodeInfo>();
        NodeInfo init;
        NodeInfo goal;

        //Collisions
        private GameObject carDetected;
        private float safetyDist = 3f;

        //Step_on
        private int seed;
        private float timer = 60.0f;
        private bool isOvertaking = false;
        private bool stopped = false;

        public void Awake()
        {
            wheelDrive = GetComponent<WheelDrive_v2>();
            currentGasoline = Random.Range(20, maxGasoline);
            //currentGasoline = 20;
            SetInitWaypoint();
        }

        public void Update()
        {
            if(state == CarStates.STEP_ON) timer -= Time.deltaTime;
        }

        public void Start()
        {
            StartCoroutine(RandomSeed());
            IEnumerator RandomSeed()
            {
                while (true)
                {
                    seed = Random.Range(0, 100);
                    Debug.Log("New seed");
                    yield return new WaitForSeconds(20);
                }
            }
        }

        #region CIRCULATE NORMAL
        public void Circulate()
        {
            Debug.Log("CIRCULATE");
            state = CarStates.CIRCULATE;

            initMaxSpeed = wheelDrive.maxSpeed;
            nextNode.waypoint = -1;
            //SetInitWaypoint();
            stopped = false;

            if (currentCorutine != null)
            {
                StopCoroutine(currentCorutine);
                currentCorutine = null;
                //setdestination
            }

            currentCorutine = StartCoroutine(CirculateCoroutine());

            IEnumerator CirculateCoroutine()
            {
                while (true)
                {
                    //if (OnRefuelEntry()) yield break;
                    wheelDrive.maxSpeed = initMaxSpeed;
                    float acc = 0.5f;
                    float brake = 0.0f; //freno

                    if(LowFuelTank())
                    {
                        yield break;
                        //state = CarStates.REFUEL;
                        //init = new NodeInfo(actualNode.segment, actualNode.waypoint, actualNode.transf);
                        //goal = new NodeInfo(2, 0, trafficSystem.segments[2].waypoints[0].transform);
                    }
                    if (GoFast()) yield break;
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
                        GameObject obj;
                        yield return new WaitUntil(() => { return !raycast.CarInRadius(out obj); });
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

                    if (RedTrafficLight()) 
                    {
                        acc = 0.0f;
                        brake = 1.0f;
                        wheelDrive.maxSpeed = Mathf.Max(wheelDrive.maxSpeed / 2f, wheelDrive.minSpeed);
                    }

                    wheelDrive.Move(acc, brake);
                    

                    //GASOLINE
                    if (gasolineDecreaser <= 0)
                    {
                        currentGasoline -= 1;
                        gasolineDecreaser = 1000;
                    } else gasolineDecreaser--;

                    yield return null;
                }
            }

        }
        #endregion

        #region REFUEL
        public void GoFuelStation()
        {
            state = CarStates.REFUEL;

            initMaxSpeed = wheelDrive.maxSpeed;
            nextNode.waypoint = -1;

            if (currentCorutine != null)
            {
                StopCoroutine(currentCorutine);
                currentCorutine = null;
            }

            currentCorutine = StartCoroutine(GoingCorutine());

            IEnumerator GoingCorutine()
            {
                init = new NodeInfo(actualNode.segment, actualNode.waypoint, actualNode.transf);
                goal = new NodeInfo(2, 0, trafficSystem.segments[2].waypoints[0].transform);

                while (true)
                {
                    if (OnRefuelEntry()) yield break;
                    wheelDrive.maxSpeed = initMaxSpeed;
                    float acc = 0.5f;
                    float brake = 0.0f; //freno

                    if (IsInWaypoint())
                    {
                        NodeInfo aux = GetNextWaypoint(init, goal);
                        actualNode.segment = aux.segment;
                        actualNode.waypoint = aux.waypoint;
                        actualNode.transf = trafficSystem.segments[actualNode.segment].waypoints[actualNode.waypoint].transform;
                    }
                    if (WrongDirection(actualNode.transf.position))
                    {
                        //Slow down
                        acc = 0.1f;
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

                    if (RedTrafficLight()) //CUIDADO QUE PILLA LOS SEMAFOROS A LOS LADOS
                    {
                        //Debug.Log("STOP");
                        acc = 0.0f;
                        brake = 1.0f;
                        wheelDrive.maxSpeed = Mathf.Max(wheelDrive.maxSpeed / 2f, wheelDrive.minSpeed);
                    }
                    float hitDist;
                    if(CarDetected(out hitDist))
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

                    wheelDrive.Move(acc, brake);

                    //GASOLINE
                    if (gasolineDecreaser <= 0)
                    {
                        currentGasoline -= 1;
                        gasolineDecreaser = 1000;
                    }
                    else gasolineDecreaser--;

                    yield return null;
                }
            }
        }

        public void RefuelTank()
        {
            if (currentCorutine != null)
            {
                Debug.Log("ON FUEL STATION (break coroutine)");
                StopCoroutine(currentCorutine);
                currentCorutine = null;
            }

            currentCorutine = StartCoroutine(Refueling());

            IEnumerator Refueling()
            {
                while (true)
                {
                    if (OnRefuelZone())
                    {
                        wheelDrive.maxSpeed = Mathf.Max(wheelDrive.maxSpeed / 2f, wheelDrive.minSpeed);
                        wheelDrive.Move(0, 1);
                        yield return new WaitForSeconds(10);
                        currentGasoline = maxGasoline;
                        wheelDrive.maxSpeed = initMaxSpeed;
                        actualNode.segment = 6;
                        actualNode.waypoint = 1;
                        actualNode.transf = trafficSystem.segments[actualNode.segment].waypoints[actualNode.waypoint].transform;
                        yield break;
                    }
                    
                    wheelDrive.maxSpeed = initMaxSpeed;
                    float acc = 0.5f;
                    float brake = 0.0f;

                    if (WrongDirection(FuelStation.position))
                    {
                        //Slow down
                        Debug.Log("Rotating to fuel zone");
                        acc = 0.1f;
                        wheelDrive.maxSpeed = Mathf.Max(wheelDrive.maxSpeed / 2f, wheelDrive.minSpeed);
                        StartCoroutine(Rotate());
                        IEnumerator Rotate()
                        {
                            while (true)
                            {
                                transform.rotation = Quaternion.Slerp(transform.rotation, rotationAngle, Time.deltaTime * 1);
                                if (!WrongDirection(FuelStation.position)) //direccion corregida
                                {
                                    rotationAngle = Quaternion.identity;
                                    yield break;
                                }

                                yield return null;
                            }
                        }
                    }
                    float hitDist;
                    if (CarDetected(out hitDist)) brake = 1.0f;

                    wheelDrive.Move(acc, brake);

                    yield return null;
                }
            }
        }
        #endregion

        #region STEP_ON
        public void StepOn()
        {
            Debug.Log("STEP_ON");
            state = CarStates.STEP_ON;

            timer = 60.0f;
            initMaxSpeed = wheelDrive.maxSpeed;
            nextNode.waypoint = -1;
            //SetInitWaypoint();

            if (currentCorutine != null)
            {
                StopCoroutine(currentCorutine);
                currentCorutine = null;
                //setdestination
            }

            currentCorutine = StartCoroutine(StepOnCorutine());

            IEnumerator StepOnCorutine()
            {
                
                while (true)
                {
                    //if (OnRefuelEntry()) yield break;
                    wheelDrive.maxSpeed = initMaxSpeed*2;
                    float acc = 0.8f;
                    float brake = 0.0f; //freno

                    if (LowFuelTank())
                    {
                        yield break;
                        //state = CarStates.REFUEL;
                        //init = new NodeInfo(actualNode.segment, actualNode.waypoint, actualNode.transf);
                        //goal = new NodeInfo(2, 0, trafficSystem.segments[2].waypoints[0].transform);
                    }
                    if (Timer()) yield break;
                    if (IsInWaypoint())
                    {
                        actualNode = ChooseDirection();
                        //ChooseDirection();

                    }
                    GameObject other;
                    //float hitDist;
                    if (raycast.CarInRadius(out other))
                    {
                        //WheelDrive_v2 other = carDetected.GetComponent<WheelDrive_v2>();
                        float dotFront = Vector3.Dot(transform.forward, other.transform.forward);
                        float hitDist = Vector3.Distance(transform.position, other.transform.position);
                        //Si hay coche delante ADELANTA (por la izq)
                        if (other.GetComponent<WheelDrive_v2>().maxSpeed <= wheelDrive.maxSpeed && dotFront > 0.8f && !isOvertaking)
                        {
                            Debug.Log("Adelantar");
                            
                            //transform.LookAt(new Vector3 (transform.position.x, transform.position.y + 45, transform.position.z));
                            acc = 1.0f;
                            //transform.LookAt(new Vector3(carDetected.transform.position.x+2.5f, carDetected.transform.position.y, carDetected.transform.position.z));
                            //.LookAt(carDetected.transform.forward);
                            transform.localPosition = new Vector3(transform.localPosition.x+1.5f, transform.localPosition.y, transform.localPosition.z);
                            isOvertaking = true;
                            yield return new WaitForSeconds(1);
                        }
                        //Si estan muy cerca y van en la direcciones opuestas
                        else if (hitDist < safetyDist && dotFront <= .8f)
                        {
                            Debug.Log("AHHHHHHHHHHHHHHHHH");
                            acc = -0.5f;
                            brake = 1;
                            wheelDrive.maxSpeed = initMaxSpeed;
                            wheelDrive.maxSpeed = Mathf.Max(wheelDrive.maxSpeed / 2f, wheelDrive.minSpeed);
                            wheelDrive.Move(acc, brake);
                            yield return new WaitForSeconds(2);
                            stopped = true;
                            yield break;
                        }
                    }
                    //wheelDrive.Move(acc, brake);
                    //yield return new WaitUntil(() => { return ; });
                    
                    if(isOvertaking && !raycast.CarInRadius(out other))
                    {
                        Debug.Log("Finished overtaking");
                        isOvertaking = false;
                        acc = 0.6f;
                        transform.localPosition = new Vector3(transform.localPosition.x - 1.5f, transform.localPosition.y, transform.localPosition.z);
                        yield return new WaitForSeconds(1);
                        SetInitWaypoint();
                        carDetected = null;
                    }
                    
                    if (raycast.PedestrianDetected() != null)
                    {
                        acc = 0;
                        brake = 1;
                        wheelDrive.maxSpeed = Mathf.Max(wheelDrive.maxSpeed / 2f, wheelDrive.minSpeed);
                    }
                    if (WrongDirection(actualNode.transf.position) && !isOvertaking)
                    {
                        //Slow down
                        acc = 0.1f;
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

                    //GASOLINE
                    if (gasolineDecreaser <= 0)
                    {
                        currentGasoline -= 2;
                        gasolineDecreaser = 1000;
                    }
                    else gasolineDecreaser--;

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

        

        private bool RedTrafficLight()
        {
            Vector3 centreCar = new Vector3(transform.position.x, transform.position.y + 0.5f, transform.localPosition.z);
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
        private bool CheckOvertaken()
        {
            Vector3 backCar = new Vector3(transform.position.x, transform.position.y + 0.5f, transform.position.z);
            RaycastHit hit;
            Debug.DrawRay(backCar, transform.right * 5f, new Color(1, 0, 0, 0.5f));
            if (Physics.Raycast(backCar, transform.right, out hit, 5f))
            {
                if (hit.collider.CompareTag("AutonomousVehicle"))
                {
                    return false;
                }
            }
            return true;
        }

        private NodeInfo GetNextWaypoint(NodeInfo current, NodeInfo goal)
        {
            Debug.Log("Search path");
            //Point searchResult = SearchPath(new NodeInfo(current.segment, current.waypoint, current.transf), goal);
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

            while(path.Any())
            {
                if (path[0].actual.segment == goal.segment && path[0].actual.waypoint == goal.waypoint)
                {
                    Debug.Log("RETURN");
                    return path[0];
                }

                List<NodeInfo> hijos = path[0].actual.Neightbors(trafficSystem);
                //Debug.Log("Hijos nodo (S" + path[0].actual.SegmentID+", W" + path[0].actual.WaypointID+"): ");

                foreach(NodeInfo hijo in hijos)
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

        #region Perceptions
        public bool LowFuelTank()
        {
            return currentGasoline < 20;
        }

        public bool TankFull()
        {
            return currentGasoline == maxGasoline;
        }

        public bool OnRefuelEntry()
        {
             if (Vector3.Distance(transform.position, goal.transf.position) < 1.2f)
             {
                 return true;
             }
            return false;
        }

        public bool OnRefuelZone()
        {
            if (Vector3.Distance(transform.position, FuelStation.position) < 1.0f) return true;
            return false;
        }

        public bool GoFast()
        {
            if(seed == 21 || seed == 3 || seed == 74)
            {
                Debug.Log("A correr");
                return true;
            }
            return false;
        }
        public bool Timer()
        {
            if (timer <= 0.0f) return true;
            return false;
        }

        public bool Stopped()
        {
            return stopped;
        }
        
        #endregion

        #region BT
        public Status OnEntryRefuelStation()
        {
            if (OnRefuelEntry())
            {
                Debug.Log("Return success");
                return Status.Success;
            }
            return Status.Running;
        }
        public Status FinhishedRefueling()
        {
            if (currentGasoline == maxGasoline) return Status.Success;
            return Status.Running;
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
