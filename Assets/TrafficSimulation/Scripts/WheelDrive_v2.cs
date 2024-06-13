using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Unity.VisualScripting;
using UnityEngine;

namespace Assets.TrafficSimulation.Scripts
{
    public class WheelDrive_v2: MonoBehaviour
    {
        
        private WheelCollider[] wheels;
        public GameObject leftWheelShape;
        public GameObject rightWheelShape;

        private float maxTorque = 200f;
        private float brakeTorque = 100000f;
        private float downForce = 100f;
        private float currentSteering = 0f;
        private float steeringLerp = 5f;

        public float maxSpeed = 30;
        public float minSpeed = 5;


        public bool animateWheels = true;

        void OnEnable()
        {
            wheels = GetComponentsInChildren<WheelCollider>();

            foreach(var wheel in wheels)
            {
                if(leftWheelShape != null && wheel.transform.localPosition.x < 0) 
                {
                    var ws = Instantiate(leftWheelShape);
                    ws.transform.parent = wheel.transform;
                } else if(rightWheelShape != null && wheel.transform.localPosition.x > 0)
                {
                    var ws = Instantiate(rightWheelShape);
                    ws.transform.parent = wheel.transform;
                }
                wheel.ConfigureVehicleSubsteps(10, 1, 1);
            }
        }

        public void Move(float _acceleration, float _brake)
        {
            Rigidbody rb = this.GetComponent<Rigidbody>();
            float torque = maxTorque * _acceleration; //fuerza de torsion (que giren las ruedas)
            float handBrake = _brake > 0 ? brakeTorque : 0; //freno

            foreach (var wheel in wheels)
            {
                if (wheel.transform.localPosition.z < 0) wheel.brakeTorque = handBrake;
                wheel.motorTorque = torque;


                // Update visual wheels if allowed
                if (animateWheels)
                {
                    Quaternion q;
                    Vector3 p;
                    wheel.GetWorldPose(out p, out q);

                    Transform shapeTransform = wheel.transform.GetChild(0);
                    shapeTransform.position = p;
                    shapeTransform.rotation = q;
                }
            }

            //Apply speed
            float s = GetSpeedUnit(rb.velocity.magnitude);
            if (s > maxSpeed) rb.velocity = GetSpeedMS(maxSpeed) * rb.velocity.normalized;


            //Apply downforce
            rb.AddForce(-transform.up * downForce * rb.velocity.magnitude);
        }

        public void WheelsRotation(float angle)
        {

            foreach (var wheel in wheels)
            {
                if (wheel.transform.localPosition.z > 0) wheel.steerAngle = angle;
            }
        }

        public float GetSpeedMS(float _s)
        {
            return _s / 3.6f;
        }

        public float GetSpeedUnit(float _s)
        {
            return _s * 3.6f;
        }
    }
}
