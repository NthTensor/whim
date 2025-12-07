//! Whim is a simple reëntrant HTN planner, based loosely on [IPyHOP].
//!
//! [IPyHOP]: https://github.com/YashBansod/IPyHOP
//!
//! # Usage
//!
//! To use whim, you will need to set up [`Model`] and [`Task`] types for your
//! planning domain. Once that's done, you simply need to pass a [`EstrMap`]
//! naming all valid tasks into [`Planner::new`].
//!
//! Using a planner should look something like this:
//! ```
//! # fn exec_plan() -> bool {
//! # use whim::*;
//! # use estr::EstrMap;
//! # #[derive(Clone)]
//! # struct MyModel;
//! # impl MyModel {
//! #     fn new() -> MyModel { MyModel }
//! #     fn simulate_action(&mut self, task_ref: TaskRef<()>) -> bool { true }
//! # }
//! # struct MyResolver;
//! # impl Resolver<()> for MyResolver {
//! #   fn next_method(&mut self) -> Option<Vec<TaskRef<()>>> { None }
//! # }
//! # struct MyTask;
//! # impl Task for MyTask {
//! #     type Data = ();
//! #     type Model = MyModel;
//! #     type Resolver = MyResolver;
//! #     fn resolve(&self, _data: &()) -> Option<MyResolver> { None }
//! #     fn is_valid(&self, _data: &(), _model: &MyModel) -> bool { true }
//! #     fn apply_effects(&self, _data: &(), _model: &mut MyModel) {}
//! # }
//! # let mut my_tasks: EstrMap<MyTask> = EstrMap::default();
//! # my_tasks.insert("root_task".into(), MyTask);
//! // Create your world model.
//! let mut model = MyModel::new();
//! // Create a planner (assuming you have already set up `my_tasks`).
//! let root_task = TaskRef {
//!     name: "root_task".into(),
//!     data: (),
//! };
//! let mut planner = Planner::new(my_tasks, root_task);
//!
//! // Start a run/simulate loop.
//! loop {
//!     // Poll the planner for the next action. This may result in a replan, if
//!     // the ground-truth model state differs significantly from what the planner
//!     // expected to see.
//!     match planner.run(&model) {
//!         // If the planner returns an action, you execute it.
//!         Outcome::Action(task) => {
//!             // This performs the action, and mutates the world according to
//!             // it's actual observed effects. If the action turns out to fail or be impossible,
//!             // we return false.
//!             let success = model.simulate_action(task);
//!             // If the simulation encountered a problem running that task,
//!             // inform the planner that it needs to create a new plan.
//!             if !success {
//!                 planner.replan(&model)
//!             }
//!         },
//!         // This indicates we ran the plan to completion. In this example, we simply exit.
//!         Outcome::Complete => return true,
//!         // This indicates that the plan cannot currently progress, given the
//!         // provided world state. In this example, we'll treat that as an
//!         // unrecoverable error and simply exit as well.
//!         Outcome::Failure => return false,
//!     }     
//! }
//! # }
//! ```

#![no_std]

extern crate alloc;

use alloc::{vec, vec::Vec};
use estr::{Estr, EstrMap};

/// Constituent tasks that will be assembled into a plan.
pub trait Task {
    /// Arbitrary data that can configure a task. This is produced by the
    /// resolver, and consuemd by `is_valid` and `apply_effects`.
    type Data: Clone;

    /// The model which is used while planning.
    type Model: Clone;

    /// The resolver type for this task type.
    type Resolver: Resolver<Self::Data>;

    /// Creates a [`Resolver`] for the task, using the provided data, which is
    /// used by the planner to break the task into sub-tasks.
    ///
    /// Returning `None` indicates that this task is a "primitive actions" which
    /// can be directly acomplished. Only tasks that return `None` here will be
    /// returned by [`Planner::run`].
    fn resolve(&self, data: &Self::Data) -> Option<Self::Resolver>;

    /// Returns true if the task can be accomplished under the conditions
    /// described by the model.
    fn is_valid(&self, data: &Self::Data, model: &Self::Model) -> bool;

    /// Modifies the model to describe the expected outcome of succesfully
    /// performing the task.
    fn apply_effects(&self, data: &Self::Data, model: &mut Self::Model);
}

/// Breaks down tasks into sub-tasks.
pub trait Resolver<D> {
    /// Produces a sequence of sub-tasks (called a "method") which can be
    /// used to acomplish the some task.
    ///
    /// This should never return the same sequence of sub-tasks more than once.
    /// The order in which you return solutions also matters; The planner will
    /// select the first set of sub-tasks that it expects to succeed.
    ///
    /// When writing resolvers:
    /// + Try to return higher priorty methods before lower-priority ones.
    /// + If you can estimate success, try to return likely successes first.
    /// + Otherwise, return solutions as randomly as possible.
    fn next_method(&mut self) -> Option<Vec<TaskRef<D>>>;
}

/// A reference to a specific task.
pub struct TaskRef<D> {
    /// The name of the task in the planner's dictionary.
    pub name: Estr,
    /// Arbitrary data which can be used to customize the behavior of the
    /// referenced task.
    pub data: D,
}

// A node in a plan. The plan is a tree of nodes, stored as a
// depth-first-preorder stack.
struct Node<T>
where
    T: Task,
{
    // The index of the parent node in the stack. We don't store children
    // because the algorithm only calls for depth-first-preorder traversal, and
    // that's just a linear scan of the stack.
    parent: usize,
    // The name of a task, used as a key into the planner's task set.
    task_name: Estr,
    // The data associated with this instance of the task.
    task_data: T::Data,
    // Stored model. This is inserted when the node is created, and used during
    // backtracking to reset the model to an earlier state.
    model: T::Model,
    // TODO
    resolver: Option<T::Resolver>,
    // Pending sub-tasks waiting to be added to the plan. These are taken from
    // the method we are currently exploring, and stored in reverse priority
    // order (so that `pop` removes the next task that should be added to the
    // plan).
    sub_tasks: Vec<TaskRef<T::Data>>,
}

/// An online lazy-lookahead planner.
pub struct Planner<T>
where
    T: Task,
{
    // The set of tasks used to build the plan. This is never modified by the
    // planner.
    tasks: EstrMap<T>,
    // TODO
    root_task: TaskRef<T::Data>,
    // A tree of nodes, stored as a depth-first-preorder stack. The plan is only
    // modified from the end, by pushing and popping nodes.
    plan: Vec<Node<T>>,
    // The index of the first unexecuted task.
    index: usize,
    // True when the plan is valid, and can be executed.
    valid: bool,
}

/// The outcome of running the planner.
pub enum Outcome<D> {
    /// This action should be executed. Call `run` again when it is finished to
    /// get a new action.
    Action(TaskRef<D>),
    /// The plan was completed. Calling `run` again will create a fresh plan and
    /// return the first action in that plan.
    Complete,
    /// The planner was unable to generate a valid plan for the current
    /// situation.
    Failure,
}

impl<T> Planner<T>
where
    T: Task,
{
    /// Creates a new planner.
    pub fn new(tasks: EstrMap<T>, root_task: TaskRef<T::Data>) -> Self {
        Planner {
            tasks,
            root_task,
            plan: vec![],
            index: 0,
            valid: false,
        }
    }

    /// Runs the planner as part of an online look-ahead loop. This will return
    /// the next valid action in the plan, replanning partially or completely
    /// when required.
    ///
    /// By default, the planner assumes that the action it returns is executed,
    /// and will progress the plan. If the action cannot be executed
    /// successfully, call `replan` to adjust the plan to account for the failure.
    pub fn run(&mut self, model: &T::Model) -> Outcome<T::Data> {
        if self.plan.is_empty() {
            let resolver = self.tasks[&self.root_task.name].resolve(&self.root_task.data);
            self.plan.push(Node {
                parent: 0,
                task_name: self.root_task.name,
                task_data: self.root_task.data.clone(),
                model: model.clone(),
                sub_tasks: vec![],
                resolver,
            });
        }

        let mut working_model = model.clone();
        match self.find_next_action(&mut working_model) {
            Outcome::Failure => {
                // If we encounter a failure, replan and look again. We need to
                // use the "dirty" working model, because it will have the
                // mutations from the successful part of the plan.
                self.replan(&mut working_model);

                // Create a fresh working model and try again.
                let mut working_model = model.clone();
                self.find_next_action(&mut working_model)
            }
            outcome => outcome,
        }
    }

    /// Resets the planner. The next time `run` is called, a fresh plan will be
    /// generated.
    pub fn reset(&mut self) {
        self.plan.truncate(1);
        self.index = 0;
        self.plan = vec![];
        self.valid = false;
    }

    /// Finds the next action in the present plan. This returns failure if the
    /// action's conditions are not met, or complete if there is no next action.
    fn find_next_action(&mut self, model: &mut T::Model) -> Outcome<T::Data> {
        if self.valid {
            let mut next_action = None;
            // We need to ensure the plan is still valid under this model, so we
            // walk the entire plan and check each action, applying effects as
            // we go. We also track the first action we encounter, so that we
            // can return it.
            for i in self.index..self.plan.len() {
                let node = &self.plan[i];
                let task = &self.tasks[&node.task_name];
                if task.is_valid(&node.task_data, &model) {
                    task.apply_effects(&node.task_data, model);

                    // Record the first primitive action we come across.
                    if node.resolver.is_none() && next_action.is_none() {
                        next_action = Some(i);
                    }
                } else {
                    return Outcome::Failure;
                }
            }
            // If we found a next action, and the plan is still valid, return
            // it. If we failed to find a next action, the plan must be
            // complete.
            if let Some(i) = next_action {
                self.index = i + 1;
                let node = &self.plan[i];
                Outcome::Action(TaskRef {
                    name: node.task_name,
                    data: node.task_data.clone(),
                })
            } else {
                // Reset when we complete to case a replan.
                self.reset();
                Outcome::Complete
            }
        } else {
            // An invalid plan has no next action.
            Outcome::Failure
        }
    }

    /// Informs the planner that the last action (as provided by
    /// [`Planner::next`]) could not be executed.
    pub fn replan(&mut self, model: &T::Model) {
        // If we previously had a valid plan, try to fix it.
        if self.valid {
            // First we discard the failed portion of the plan -- the bit that
            // we have not yet run.
            self.unrefine_plan();
            // Then we try replanning it repeatedly, reconsidering a slightly
            // larger portion of the already completed bit each time.
            //
            // If the length of the plan will reach one only if we backtrack all
            // the way up to the root -- which means we are effectively
            // reconsidering the entire plan, and we can stop trying to do
            // partial replans.
            while self.plan.len() >= 1 {
                // Backtrack one level away from the failure, which means we
                // reconsider a set of actions we already took.
                self.backtrack();
                // Overwrite the last node's model with the current model, so
                // that it will generate a plan that is valid to start from the
                // current moment.
                self.plan.last_mut().unwrap().model = model.clone();
                // Try replaning from the last node.
                if self.plan() {
                    // If it's valid, we can exit.
                    self.valid = true;
                    return;
                }
            }
            // If we get here, the only thing left to try is a full from-scratch
            // replan.
        }
        // Try to create a totally fresh plan, from scratch, with the new model.
        self.reset();
        self.plan[0].model = model.clone();
        self.valid = self.plan();
    }

    /// Tries to produce a valid plan. Returns true on success and false on
    /// failure.
    ///
    /// If this function is called with a partial plan (a valid plan that has
    /// been truncated, or an invalid plan that was never completed) it will
    /// attempt resume planning where it left off.
    ///
    /// This function will never modify tasks before `index`, as it assumes
    /// these have already been executed. It will instead find plans that could
    /// be valid ways to complete the already started sequence of actions.
    fn plan(&mut self) -> bool {
        // The run index must be within the plan.
        self.index = usize::min(self.plan.len() - 1, self.index);
        loop {
            // The plan is built incrementally by checking and expanding the
            // last node in the plan.
            let node = self.plan.last_mut().expect("plan must never be empty");
            let mut model = node.model.clone();
            let task = &self.tasks[&node.task_name];

            // Try to validate and refine the node.
            if task.is_valid(&node.task_data, &node.model) {
                if let Some(resolver) = &mut node.resolver {
                    if let Some(mut sub_tasks) = resolver.next_method() {
                        sub_tasks.reverse();
                        node.sub_tasks = sub_tasks;
                        task.apply_effects(&node.task_data, &mut model);
                    } else if self.index < self.plan.len() {
                        self.backtrack();
                    } else {
                        return false;
                    }
                } else {
                    task.apply_effects(&node.task_data, &mut model);
                }
            } else if self.index < self.plan.len() {
                // If the task isn't valid, backtrack to the parent.
                self.backtrack();
            } else {
                // If we are at the run index (the first unexecuted task) we are
                // unable to backtrack and must return failure.
                return false;
            }

            // Walk back up the tree looking for a node to refine. Because the
            // plan is stored in depth-first-preorder, this is a reversed
            // depth-first-preorder search.
            let mut i = self.plan.len() - 1;
            loop {
                if let Some(task_ref) = self.plan[i].sub_tasks.pop() {
                    // If we find a node that has pending tasks, we pop one of
                    // those tasks and add it at the end of the tree -- this is
                    // fundamentally what causes the tree to be stored in
                    // depth-first-preorder.
                    let resolver = self.tasks[&task_ref.name].resolve(&task_ref.data);
                    let node = Node {
                        parent: i,
                        task_name: task_ref.name,
                        task_data: task_ref.data,
                        model,
                        sub_tasks: vec![],
                        resolver,
                    };
                    // Since this is added at the end of the tree, this will be
                    // the node for the next iteration.
                    self.plan.push(node);
                    break;
                } else {
                    // If the node doesn't have pending tasks (meaning it's an
                    // action or it's been fully refined) move to the parent. If
                    // we are at the run index (the first unexecuted task) the
                    // plan is fully refined and we can return success.
                    if i == self.index {
                        return true;
                    } else {
                        i -= 1;
                    }
                }
            }
        }
    }

    /// Truncates a finished but partially-executed plan, discarding all
    /// not-yet-executed nodes so that the latter half of the plan can be re-evaluated.
    fn unrefine_plan(&mut self) {
        // Split off the unexecuted part of the plan.
        let invalid_nodes = self.plan.split_off(self.index);
        // Any nodes that are children of nodes still in the main plan must be
        // added back as unrefined sub-tasks, so that they can be used in replanning.
        for node in invalid_nodes.into_iter() {
            if let Some(parent) = self.plan.get_mut(node.parent) {
                parent.sub_tasks.push(TaskRef {
                    name: node.task_name,
                    data: node.task_data,
                });
            }
        }
    }

    /// Returns to the parent node, and discards it's current children along
    /// with the method it used to generate them.
    fn backtrack(&mut self) {
        // Grab the current node.
        let node = self.plan.last().expect("plan must never be empty");
        // Look up the parent.
        let parent = node.parent;
        // Truncate the plan after the parent. The plan is stored in
        // depth-first-preorder, so this has the effect of removing all the
        // children of the parent.
        self.plan.truncate(parent + 1);
        // Remove any stale sub-tasks still on the parent, since the method that
        // provided them has failed.
        self.plan[parent].sub_tasks.clear();
    }
}
