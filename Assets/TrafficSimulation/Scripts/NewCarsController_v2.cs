using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using BehaviourAPI.Core;
using BehaviourAPI.StateMachines;
using BehaviourAPI.BehaviourTrees;
using Assets.TrafficSimulation.Scripts;

namespace TrafficSimulation.Scripts
{
    public class NewCarsController_v2 : MonoBehaviour
    {
        public struct Node
        {
            public int segment;
            public int waypoint;
            public Transform transf;
        }

        public TrafficSystem trafficSystem;
        private Node intersectionNode;
        private Node nextNode;

        private bool isInIntersection = false;
        //private bool isRotating = false;
        //private Vector3 desiredDir;
        private Quaternion rotateAngle=Quaternion.identity;

        //Car
        WheelDrive_v2 wheelDrive;
        private float initMaxSpeed;

        void Awake()
        {
            wheelDrive = this.GetComponent<WheelDrive_v2>();
            initMaxSpeed = wheelDrive.maxSpeed;
            nextNode.waypoint = -1;
            SetInitWaypoint();
        }

        // Start is called before the first frame update
        void Start()
        {
            if (trafficSystem == null)
                return;

            //SetInitWaypoint();
        }

        // Update is called once per frame
        void Update()
        {

        }

        private void OnTriggerEnter(Collider other)
        {
            if (other.gameObject.CompareTag("Intersection"))
            {
                Debug.Log("Entrando en interseccion " + other.GetComponent<Intersection>().id);
                isInIntersection = true;
            }
        }

        private void OnTriggerExit(Collider other)
        {
            if (other.gameObject.CompareTag("Intersection"))
            {
                Debug.Log("Saliendo de interseccion");
                isInIntersection = false;

            }
        }

        #region FSM States
        public void ToMove()
        {
            StartCoroutine(Move());
        }
        
        private IEnumerator Move()
        {
            while(true)
            {
                Debug.Log("Velocity: " + wheelDrive.maxSpeed);

                wheelDrive.maxSpeed = initMaxSpeed;
                float acc = 0.5f;
                float brake = 0; //freno
                //float steering = 0f;

                if (rotateAngle != Quaternion.identity)
                {
                    Debug.Log("Rotate");
                    acc = 0.0f;
                }

                wheelDrive.Move(acc, brake);
                /*
                if (isInIntersection)
                {
                    Debug.Log("STOP");
                    yield break;
                }*/
                yield return null;
            }
        }

        public void ToTurn()
        {
            StartCoroutine(Rotate());

        }

        private IEnumerator Rotate()
        {
            while(true)
            {
                /*
                Vector3 desiredVel = trafficSystem.segments[intersectionNode.segment].waypoints[intersectionNode.waypoint].transform.position - this.transform.position;
                float steering = Mathf.Clamp(this.transform.InverseTransformDirection(desiredVel.normalized).x, -1f, 1f);
                Debug.Log("Steering = " + steering);
                wheelDrive.WheelsRotation(steering);

                Vector3 direction = (intersectionNode.transf.position - transform.position).normalized;
                if (Mathf.Abs(1.0f - Vector3.Dot(direction, transform.forward)) < 0.1f) yield break;*/
                transform.rotation = Quaternion.Slerp(transform.rotation, rotateAngle, Time.deltaTime * 1);
                if (!DirectionIsNotCorrect())
                {
                    rotateAngle = Quaternion.identity;
                    transform.LookAt(intersectionNode.transf.position);
                    yield break;
                }
                yield return null;
            }
        }

        public void ChooseDirection()
        {
            Debug.Log("Actual: Segmento " + intersectionNode.segment + ", Waypoint " + intersectionNode.waypoint);

            nextNode.waypoint = intersectionNode.waypoint + 1;
            nextNode.segment = intersectionNode.segment;

            if (nextNode.waypoint >= trafficSystem.segments[intersectionNode.segment].waypoints.Count)
            {
                nextNode.waypoint = 1;
                nextNode.segment = GetNextSegmentId();
            }
            nextNode.transf = trafficSystem.segments[nextNode.segment].waypoints[nextNode.waypoint].transform;
            Debug.Log("Nueva direccion: Segmento " + nextNode.segment + ", Waypoint " + nextNode.waypoint);
        }

        //Transitions
        public bool IsInWaypoint()
        {
            return Vector3.Distance(this.transform.position, intersectionNode.transf.position) < 1.0f;
        }

        public bool DirectionChosen()
        {
            //forward es un 1/-1 en direccion en la q mira
            bool aux = false;
            if (intersectionNode.segment != nextNode.segment || intersectionNode.waypoint != nextNode.waypoint)
            {
                aux = true;
                intersectionNode = nextNode;
            }
            return aux;
        }

        public bool DirectionIsNotCorrect()
        {
            Vector3 direction = (intersectionNode.transf.position - transform.position).normalized;
            float angle = Mathf.Abs(1.0f - Vector3.Dot(direction, transform.forward));
            if(angle > 0.05f)
            {
                rotateAngle = Quaternion.LookRotation((intersectionNode.transf.position - transform.position).normalized);
                return true;
            }
            return false;
        }

        #endregion



        private void SetInitWaypoint()
        {
            //Find current target
            foreach (Segment segment in trafficSystem.segments)
            {
                if (segment.IsOnSegment(this.transform.position))
                {
                    intersectionNode.segment = segment.id;

                    //Find nearest waypoint to start within the segment
                    float minDist = float.MaxValue;
                    for (int j = 0; j < trafficSystem.segments[intersectionNode.segment].waypoints.Count; j++)
                    {
                        float d = Vector3.Distance(this.transform.position, trafficSystem.segments[intersectionNode.segment].waypoints[j].transform.position);

                        //Only take in front points
                        Vector3 lSpace = this.transform.InverseTransformPoint(trafficSystem.segments[intersectionNode.segment].waypoints[j].transform.position);
                        if (d < minDist && lSpace.z > 0)
                        {
                            minDist = d;
                            intersectionNode.waypoint = j;
                        }
                    }
                    break;
                }
            }
            intersectionNode.transf = trafficSystem.segments[intersectionNode.segment].waypoints[intersectionNode.waypoint].transform;
            Debug.Log("Nearest waypoint: " + intersectionNode.segment + " segment, " + intersectionNode.waypoint + " waypoint --> "+intersectionNode.transf.position);
        }

        int GetNextSegmentId()
        {
            if (trafficSystem.segments[intersectionNode.segment].nextSegments.Count == 0)
                return 0;
            int c = UnityEngine.Random.Range(0, trafficSystem.segments[intersectionNode.segment].nextSegments.Count);
            return trafficSystem.segments[intersectionNode.segment].nextSegments[c].id;
        }
    }
}
