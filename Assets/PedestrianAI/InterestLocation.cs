using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InterestLocation : MonoBehaviour
{
    public bool isOccupied = false;

    public bool Occupy()
    {
        if (!isOccupied)
        {
            isOccupied = true;
            return true;
        }
        return false;
    }

    public void Vacate()
    {
        isOccupied = false;
    }
}
