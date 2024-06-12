using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using BehaviourAPI.Core;
using BehaviourAPI.StateMachines;
using BehaviourAPI.BehaviourTrees;
using Unity.VisualScripting;
using Unity.VisualScripting.Antlr3.Runtime;
using UnityEngine.UIElements;

namespace TrafficSimulation.Scripts
{
    public class NewCarsController : MonoBehaviour
    {
        public struct Target
        {
            public int segment;
            public int waypoint;
        }

        public TrafficSystem trafficSystem;
        public Transform carSensorRadius;

        //private GameObject carDetected;
        private bool carSensor = false;

        //Car movement
        private WheelDrive wheelDrive;
        private float initMaxSpeed;

        private float acc;
        private float brake;
        private float steering;
        private float maxSteerAngle = 45f;
        private float epsilon = 0.1f;
        private bool turning = false;
        private Quaternion lookingAt = Quaternion.identity;
        private float turnSpeed = 3;

        private bool isInInterestZone = false;

        private Target waypointIntersection;
        private Target nextWaypoint;

        public float waypointThresh = 6;

        //probando
        private float c_currentV = 0;

        private void Awake()
        {
            wheelDrive = this.GetComponent<WheelDrive>();

        }

        // Start is called before the first frame update
        void Start()
        {
            if (trafficSystem == null)
                return;

            //Init variables
            //initMaxSpeed = 30;
            initMaxSpeed = wheelDrive.maxSpeed;

            nextWaypoint.waypoint = -1;
            
            acc = 0.5f;
            brake = 0.0f;  
            steering = 0.0f;

            SetInitWaypoint();
        }

        // Update is called once per frame
        void Update()
        {
            if(!CarDetected())
            {
                if (CarNearSensor() != null)
                {
                    Debug.Log("PIPIPI");
                }
            }
        }

        //CAR BASICS FSM

        public void ToMove()
        {
            StartCoroutine(Moving());
        }

        private IEnumerator Moving()
        {
            while(true)
            {

                wheelDrive.maxSpeed = initMaxSpeed;
                acc = 1.0f;
                //¿Hay q rotar?
                /*
                if(nextWaypoint.waypoint != -1)
                {
                    Transform waypoint = trafficSystem.segments[nextWaypoint.segment].waypoints[nextWaypoint.waypoint].transform;
                    Quaternion angle = Quaternion.LookRotation(waypoint.position - this.gameObject.transform.position);
                    float speed = 0.5f;
                    this.gameObject.transform.rotation = Quaternion.Slerp(this.gameObject.transform.rotation, angle, speed);
                    
                    Transform targetTransform = trafficSystem.segments[waypointIntersection.segment].waypoints[waypointIntersection.waypoint].transform;
                    Transform futureTargetTransform = trafficSystem.segments[nextWaypoint.segment].waypoints[nextWaypoint.waypoint].transform;
                    Vector3 futureVel = futureTargetTransform.position - targetTransform.position;
                    steering = Mathf.Clamp(this.transform.InverseTransformDirection(futureVel.normalized).x, -1, 1);
                    //this.gameObject.transform.LookAt(waypoint);
                } */

                wheelDrive.Move(acc, steering, brake);

                if(isInInterestZone)
                {
                    yield break;
                }

                yield return null;
            }
            
        }

        public void TurnTo() //SLOW_DOWN
        {
            /*
            Vector3 newDirection = trafficSystem.segments[nextWaypoint.segment].waypoints[nextWaypoint.waypoint].transform.position;
            Vector3 actualDir = (newDirection-transform.position).normalized;
            if(Mathf.Abs(1.0f - Vector3.Dot(actualDir, transform.forward)) > epsilon)
            {
                //direccionObjetivo = Quaternion.LookRotation((_objetivo - transform.position).normalized);
            }
            //Steer
            Vector3 relativeVector = transform.InverseTransformPoint(trafficSystem.segments[nextWaypoint.segment].waypoints[nextWaypoint.waypoint].transform.position);
            float newSteer = relativeVector.x / relativeVector.magnitude * wheelDrive.maxAngle*10;
            foreach (WheelCollider wheel in wheelDrive.wheels)
            {
                // Steer front wheels only
                if (wheel.transform.localPosition.z > 0) wheel.steerAngle = newSteer;
            }
            turning = true;
            Vector3 newDirection = trafficSystem.segments[nextWaypoint.segment].waypoints[nextWaypoint.waypoint].transform.position;
            lookingAt = Quaternion.LookRotation((newDirection - transform.position).normalized);*/
            //Calculate if there is a planned turn
            Transform targetTransform = trafficSystem.segments[waypointIntersection.segment].waypoints[waypointIntersection.waypoint].transform;
            Transform futureTargetTransform = trafficSystem.segments[nextWaypoint.segment].waypoints[nextWaypoint.waypoint].transform;
            Vector3 futureVel = futureTargetTransform.position - targetTransform.position;
            float futureSteering = Mathf.Clamp(this.transform.InverseTransformDirection(futureVel.normalized).x, -1, 1);
            if (futureSteering > .3f || futureSteering < -.3f)
            {
                wheelDrive.maxSpeed = Mathf.Min(wheelDrive.maxSpeed, wheelDrive.steeringSpeedMax);
            }
            //Check if we need to steer to follow path
            if (acc > 0f)
            {
                Vector3 desiredVel = futureTargetTransform.position - this.transform.position;
                steering = Mathf.Clamp(this.transform.InverseTransformDirection(desiredVel.normalized).x, -1f, 1f);
            }
        }

        //TRANSITIONS
        /*
        public bool EverythingFine()
        {
            Vector3 waypoint = trafficSystem.segments[nextWaypoint.segment].waypoints[nextWaypoint.waypoint].transform.position;
            return this.gameObject.transform.LookAt(waypoint) == this.gameObject.transform.forward;
        }*/

        private void OnTriggerEnter(Collider other)
        {
            if(other.gameObject.CompareTag("Intersection"))
            {
                Debug.Log("Entrando en interseccion "+other.GetComponent<Intersection>().id);
                isInInterestZone = true;
                //acc = 0.2f;
            }
        }

        private void OnTriggerExit(Collider other)
        {
            if (other.gameObject.CompareTag("Intersection"))
            {
                Debug.Log("Saliendo de interseccion");
                isInInterestZone = false;
                //acc = 1f;
                
            }
        }

        public bool IsInInterestZone()
        {
            return isInInterestZone;
        }

        public void SlowDown()
        {
            Debug.Log("SlowDown");
            acc = 0.3f;
            initMaxSpeed/=2;

        }

        public void SpeedUp()
        {
            Debug.Log("SpeedUp");
            acc = 0.5f;
            initMaxSpeed *= 2;
        }


        public bool IsInWaypoint()
        {
            GameObject waypoint = trafficSystem.segments[waypointIntersection.segment].waypoints[waypointIntersection.waypoint].gameObject;
            //Debug.Log("Distance: "+Vector3.Distance(this.gameObject.transform.position, waypoint.transform.position));
            if (Vector3.Distance(this.gameObject.transform.position,waypoint.transform.position) < 1f)
            {
                //Debug.Log("STOP");
                //wheelDrive.maxSpeed = Mathf.Min(wheelDrive.maxSpeed / 2f, 5f);
                //wheelDrive.Move(0, 0, 1);
                //acc = 0f;
                return true;
            }
            return false;
        }

        public void SetDirection()
        {
            Debug.Log("Actual: Segmento " + waypointIntersection.segment + ", Waypoint " + waypointIntersection.waypoint);
            
            nextWaypoint.waypoint = waypointIntersection.waypoint + 1;
            nextWaypoint.segment = waypointIntersection.segment;

            if (nextWaypoint.waypoint >= trafficSystem.segments[waypointIntersection.segment].waypoints.Count)
            {
                nextWaypoint.waypoint = 1;
                nextWaypoint.segment = GetNextSegmentId();
            }
            Debug.Log("Nueva direccion: Segmento " + nextWaypoint.segment + ", Waypoint " + nextWaypoint.waypoint);

        }
        int GetNextSegmentId()
        {
            if (trafficSystem.segments[waypointIntersection.segment].nextSegments.Count == 0)
                return 0;
            int c = UnityEngine.Random.Range(0, trafficSystem.segments[waypointIntersection.segment].nextSegments.Count);
            return trafficSystem.segments[waypointIntersection.segment].nextSegments[c].id;
        }

        public bool DirectionChosen()
        {
            bool aux = false;
            if (waypointIntersection.segment != nextWaypoint.segment || waypointIntersection.waypoint != nextWaypoint.waypoint)
            {
                Debug.Log("Confirmando nuevo punto elegido");
                aux = true;
                waypointIntersection = nextWaypoint;
                return aux;
            }
            return aux;
        }

        private void SetInitWaypoint()
        {
            //Find current target
            foreach (Segment segment in trafficSystem.segments)
            {
                if (segment.IsOnSegment(this.transform.position))
                {
                    waypointIntersection.segment = segment.id;

                    //Find nearest waypoint to start within the segment
                    float minDist = float.MaxValue;
                    for (int j = 0; j < trafficSystem.segments[waypointIntersection.segment].waypoints.Count; j++)
                    {
                        float d = Vector3.Distance(this.transform.position, trafficSystem.segments[waypointIntersection.segment].waypoints[j].transform.position);

                        //Only take in front points
                        Vector3 lSpace = this.transform.InverseTransformPoint(trafficSystem.segments[waypointIntersection.segment].waypoints[j].transform.position);
                        if (d < minDist && lSpace.z > 0)
                        {
                            minDist = d;
                            waypointIntersection.waypoint = j;
                        }
                    }
                    break;
                }
            }
        }


        public bool CarDetected()
        {
            return carSensor;
        }


        private GameObject CarNearSensor()
        {
            float range = 5.0f;
            Vector3 centreCar = new Vector3(transform.position.x, transform.position.y + 0.5f, transform.position.z);

            RaycastHit hit;
            Debug.DrawRay(centreCar, transform.forward*range, Color.yellow);
            //Vector3 carPosition = car.transform.position;
            
            if (Physics.Raycast(centreCar, transform.forward, out hit, range))
            {
                if (hit.collider.CompareTag("AutonomousVehicle"))
                {
                    carSensor = true;
                    return hit.collider.gameObject;
                }
                else return null;
            }
            else return null;
        }
    }
}
