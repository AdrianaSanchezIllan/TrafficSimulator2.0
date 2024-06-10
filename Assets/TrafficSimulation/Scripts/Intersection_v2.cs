// Traffic Simulation
// https://github.com/mchrbn/unity-traffic-simulation

using System.Collections.Generic;
using UnityEngine;

namespace TrafficSimulation{
    public enum IntersectionType_v2{
        STOP,
        TRAFFIC_LIGHT
    }

    public class Intersection_v2 : MonoBehaviour
    {   
        public IntersectionType_v2 intersectionType;
        public int id;  

        //For stop only
        public List<Segment> prioritySegments;

        //For traffic lights only
        public float lightsDuration = 8;
        public float orangeLightDuration = 2;
        public List<Segment> lightsNbr1;
        public List<Segment> lightsNbr2;

        private List<GameObject> vehiclesQueue;
        private List<GameObject> vehiclesInIntersection;
        private TrafficSystem trafficSystem;
        
        public float timeSinceChange = 0;
        [HideInInspector] public int currentRedLightsGroup = 1;

        void Start(){
            if(intersectionType == IntersectionType_v2.TRAFFIC_LIGHT)
                InvokeRepeating("SwitchLights", lightsDuration, lightsDuration);
        }
        private void Update()
        {
            timeSinceChange += Time.deltaTime;
        }
        void SwitchLights(){
            timeSinceChange = 0;
            if(currentRedLightsGroup == 1) currentRedLightsGroup = 2;
            else if(currentRedLightsGroup == 2) currentRedLightsGroup = 1;            
            
            //Wait few seconds after light transition before making the other car move (= orange light)
            //Invoke("MoveVehiclesQueue", orangeLightDuration);
        }

       
    }
}
