/*******************************************************************************
*          Private SOFA components, (c) 2017 InSimo                            *
* CONFIDENTIAL SOURCE CODE. This file is the property of InSimo and should not *
* be redistributed. Commercial use is prohibited without a specific license.   *
*******************************************************************************/

#ifndef ISPHYSICS_BASE_FUNCTORTASK_H
#define ISPHYSICS_BASE_FUNCTORTASK_H

#include <MultiThreading/src/Tasks.h>
#include <MultiThreading/src/TaskSchedulerBoost.h>
#include "DependencyTask.h"
#include "initPlugin.h"

#include <utility>

ISPHYSICS_PUBLIC

namespace isphysics
{
namespace base
{

/// Task that can contain any object that can be called using its operator()
/// This can be :
///  - a regular function or method
///  - a lambda expression
///  - a std::function
///  - a class with an operator() defined
///
/// Example usage :
///     Task::Status status;
///     auto func = [](){ /* work */ };
///     FunctorTask<decltype(func)> task(std::forward<decltype(func)>(func));
///     task.enable(&status, "Function", {0.5f,0.5f,0.5f,1.f});
/// 
/// Example with argument :
///     Task::Status status;
///     auto func = [](ArgType arg){ /* work */ };
///     FunctorTask<decltype(func), ArgType> task(std::forward<decltype(func)>(func), defaultArg);
///     task.enable(&status, "Function", {0.5f,0.5f,0.5f,1.f});
/// 
/// If you want to change the argument to the functor without constructing a new FunctorTask :
///     task.setArguments(newArg);
///
/// To pass a (const) reference to the functor, use a std::reference_wrapper :
///     FunctorTask<(decltype(func), std::reference_wrapper<ArgType>> task(std::forward<decltype(func)>(func), std::ref(defaultArg));
///     FunctorTask<(decltype(func), std::reference_wrapper<const ArgType>> task(std::forward<decltype(func)>(func), std::cref(defaultArg));
/// Otherwise, calls to setArguments() will fail to compile because C++ references cannot be rebound.



template<typename TTask, typename Fn, typename... Args>
class TFunctorTask : public TTask
{
public:
    using Color  = sofa::simulation::Task::Color;
    using Status = sofa::simulation::Task::Status;

    template<typename FnT, typename... ArgsT>
    TFunctorTask(FnT&& f, ArgsT&&... args)
        : TTask()
        , m_fn(std::forward<FnT>(f))
        , m_args(std::forward<ArgsT>(args)...)
    {}

    template<typename FnT>
    TFunctorTask(FnT&& f)
        : TTask()
        , m_fn(std::forward<FnT>(f))
        // default constructed arguments
    {}

    void enable(const Status* status, const std::string& name, const Color& color = {0.5f,0.5f,0.5f,1.0f})
    {
        sofa::simulation::Task::enable(status);
        m_name = name;
        m_color = color;
    }

    template<typename... ArgsT>
    void setArguments(ArgsT&&... args)
    {
        m_args = std::tuple<Args...>(std::forward<ArgsT>(args)...);
    }
    
    bool run(sofa::simulation::WorkerThread *) override
    {
        call(std::forward<Fn>(m_fn), MakeSeq<sizeof...(Args)>(), m_args);
        return true;
    }
    
    const char* getName() const override
    {
        return m_name.c_str();
    }
    
    Color getColor() const override
    {
        return m_color;
    }
    
protected:
    Fn m_fn;
    std::tuple<Args...> m_args;
    std::string m_name;
    Color m_color;
    
private:
    // Sequence creation from variadic template (https://stackoverflow.com/a/16764595)
    template<int...> struct seq {};
    template<int Min, int Max, int... s> struct make_seq : make_seq<Min, Max-1, Max-1, s...> {};
    template<int Min, int... s> struct make_seq<Min, Min, s...>
    {
        typedef seq<s...> type;
    };
    template<int Max, int Min=0>
    using MakeSeq = typename make_seq<Min, Max>::type;

    // Unpack tuple into sequence and call functor
    template<typename Func, typename Tuple, int... s>
    void call(Func&& f, seq<s...>, Tuple&& tup)
    {
        f( std::get<s>(tup)... );
    }
};

template<typename Fn, typename... Args>
using FunctorTask = TFunctorTask< sofa::simulation::Task, Fn, Args... >;

template<typename Fn, typename... Args>
using FunctorDependencyTask = TFunctorTask< sofa::simulation::DependencyTask, Fn, Args... >;

template<typename... Args>
using FunctorTaskWithArgs = isphysics::base::FunctorTask<std::function<void(Args...)>, Args...>;

template<typename... Args>
using FunctorDependencyTaskWithArgs = isphysics::base::FunctorDependencyTask<std::function<void(Args...)>, Args...>;

/// Helper function to use when getting the type of the functor is complicated
/// Implements the object generator idiom
/// 
/// Example usage :
/// auto task = make_functorTask(func);

template<typename TTask = sofa::simulation::Task, typename Fn, typename... Args>
TFunctorTask<TTask, std::decay_t<Fn>,std::decay_t<Args>...> make_functorTask(Fn&& f, Args&&... args)
{
    return { std::forward<Fn>(f), std::forward<Args>(args)... };
}

/// Helper function to use when getting the type of the functor is complicated and the FunctorTask must be allocated dynamically
/// 
/// Example usage :
/// auto taskptr = make_new_functorTask(func);

template<typename TTask = sofa::simulation::Task, typename Fn, typename... Args>
TFunctorTask<TTask, std::decay_t<Fn>,std::decay_t<Args>...>* make_new_functorTask(Fn&& f, Args&&... args)
{
    return new TFunctorTask<TTask, std::decay_t<Fn>,std::decay_t<Args>...>( std::forward<Fn>(f), std::forward<Args>(args)... );
}

/// Helper function to create a task if needed and run it synchronously from a functor and its arguments
///
/// Example usage :
///
/// runAsTask("taskName", taskColor, taskPtr, func);

template<typename Fn, typename... Args>
void runAsTask(const std::string& name, const sofa::defaulttype::Vec4f& color, std::unique_ptr<sofa::simulation::Task>& task, Fn&& f, Args&&... args)
{
    sofa::simulation::WorkerThread* thread = sofa::simulation::WorkerThread::GetCurrent();
    if (!thread)
    {
        f(args...);
        return;
    }

    if (!task)
    {
        task.reset(base::make_new_functorTask(f, args...));
    }
    auto* functorTask = static_cast<base::FunctorTask<std::decay_t<Fn>, std::decay_t<Args>...>*>(task.get());
    functorTask->enable(thread->getCurrentStatus(), name, color);
    functorTask->setArguments(args...);
    thread->runTask(functorTask);
#if 0
    const auto ticks = sofa::helper::system::thread::CTime::getTicksPerSec();
    const float taskDurationMs = 1000.0f * task->getExecDuration() / ticks;
    std::cout << name << " : " << taskDurationMs << " ms" << std::endl;
#endif
}

/// Helper function to create a task if needed and run it synchronously from a functor and its arguments
///
/// Example usage :
///
/// runAsStealableTask(taskStatus, "taskName", taskColor, taskPtr, func);

template<typename Fn, typename... Args>
void runAsStealableTask(const sofa::simulation::Task::Status* status, const std::string& name, const sofa::defaulttype::Vec4f& color, std::unique_ptr<sofa::simulation::Task>& task, Fn&& f, Args&&... args)
{
    sofa::simulation::WorkerThread* thread = sofa::simulation::WorkerThread::GetCurrent();
    if (!thread || !status)
    {
        f(args...);
        return;
    }

    if (!task)
    {
        task.reset(base::make_new_functorTask(f, args...));
    }
    auto* functorTask = static_cast<base::FunctorTask<std::decay_t<Fn>, std::decay_t<Args>...>*>(task.get());
    functorTask->enable(status, name, color);
    functorTask->setArguments(args...);
    thread->addStealableTask(functorTask);
}


} // namespace base
} // namespace isphysics

#endif // ISPHYSICS_BASE_FUNCTORTASK_H
