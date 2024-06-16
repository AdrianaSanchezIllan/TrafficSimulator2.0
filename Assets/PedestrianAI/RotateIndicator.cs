using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RotateIndicator : MonoBehaviour
{
    [SerializeField]
    private Vector3 _rotation;
    [SerializeField]
    private float _speed;
    private void Start()
    {
        Debug.Log("Crea CAFEEE");
    }
    void Update()
    {
        transform.Rotate( _rotation * _speed * Time.deltaTime);
        //transform.Rotate(0, 5f, 0);
    }
}
