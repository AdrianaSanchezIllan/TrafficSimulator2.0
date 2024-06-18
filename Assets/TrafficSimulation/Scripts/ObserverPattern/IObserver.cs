using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public interface IObserver<T>
{
    public void UpdateObserver(T data);
}
