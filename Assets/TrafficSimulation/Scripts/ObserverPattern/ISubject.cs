using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public interface ISubject<T>
{
    public void AddObserver(IObserver<T> observer);
    public void RemoveObserver(IObserver<T> observer);
    public void NotifyObservers(T data);
}
