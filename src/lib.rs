//! Whim is a simple reëntrant HTN planner, based loosely on [IPyHOP].
//!
//! [IPyHOP]: https://github.com/YashBansod/IPyHOP
//!

#![no_std]

extern crate alloc;

use alloc::{sync::Arc, vec, vec::Vec};
use estr::{Estr, EstrMap};

#[derive(Debug)]
pub enum Task<P, C> {
    /// A primitive task contains a type that implements `Execute`, and becomes
    /// a step in the resulting plan.
    Primitive(P),
    /// A compound task contains a type that implements `Resolve`, and produces
    /// sub-tasks.
    Compound(C),
}

/// A primitive "Action" task that can be completed in a single step.
pub trait PrimitiveTask<M, D> {
    /// Returns true if the task can be accomplished under the conditions
    /// described by the model.
    fn is_valid(&self, data: &D, model: &M) -> bool;

    /// Modifies the model to describe the expected outcome of succesfully
    /// performing the task.
    fn apply_effects(&self, data: &D, model: &mut M);
}

/// Generates possible sequences of sub-tasks.
pub trait CompoundTask<M, D> {
    type Resolver: MethodResolver<M, D>;

    /// Returns a resolver that can generate methods to break down this
    /// compound task into sub-tasks.
    fn get_resolver(&self) -> Self::Resolver;
}

pub trait MethodResolver<M, D> {
    /// Produces a sequence of sub-tasks (called a "method") which can be
    /// used to acomplish the some task.
    ///
    /// This should never return the same sequence of sub-tasks more than once.
    /// The order in which you return solutions also matters; The planner will
    /// select the first set of sub-tasks it encounters which it thinks will
    /// succeed at completing the plan.
    ///
    /// This should return `Some()` only a finite number of times, then always
    /// return `None`. Returning an infinite sequence of unworkable methods
    /// may cause the planner to loop forever.
    ///
    /// When writing resolvers:
    /// + Try to return higher priorty methods before lower-priority ones.
    /// + If you can estimate success, try to return likely successes first.
    /// + Otherwise, return solutions as randomly as possible.
    fn next_method(&mut self, data: &D, model: &M) -> Option<Vec<TaskRef<D>>>;
}

/// A reference to a specific task.
#[derive(PartialEq, Debug)]
pub struct TaskRef<D> {
    /// The name of the task in the planner's dictionary.
    pub name: Estr,
    /// Arbitrary data which can be used to customize the behavior of the
    /// referenced task.
    pub data: D,
}

// A node in a plan. The plan is a tree of nodes, stored as a
// depth-first-preorder stack.
#[derive(Debug)]
struct Node<D, R> {
    // The index of the parent node in the stack. We don't store children
    // because the algorithm only calls for depth-first-preorder traversal, and
    // that's just a linear scan of the stack.
    parent: usize,
    // The name of a task, used as a key into the planner's task set.
    task_name: Estr,
    // The data associated with this instance of the task.
    task_data: D,
    /// TODO
    resolver: Option<R>,
    /// TODO
    sub_tasks: Vec<TaskRef<D>>,
}

/// An online lazy-lookahead planner.
#[derive(Debug)]
pub struct Plan<M, D, P, C>
where
    P: PrimitiveTask<M, D>,
    C: CompoundTask<M, D>,
{
    // The set of tasks used to build the plan. This is never modified by the
    // planner.
    tasks: Arc<EstrMap<Task<P, C>>>,
    // A tree of nodes, stored as a depth-first-preorder stack. The plan is only
    // modified from the end, by pushing and popping nodes.
    //
    // The structure of a plan is highly complex. It is a tree, stored in
    // depth-first preorder.
    //
    // Within the plan list, there are three important ranges:
    //
    // + [0, first_unexeucted) contains nodes that have been executed sucesfully
    //   at-least partially
    //
    // + [first_unexecuted, current_action] contains nodes that are currently
    //   being attempted for the first time
    //
    // + (current_action, ..) contains nodes that have yet to be attempted
    //
    // Furthermore, while a plan is being created or checked, the `models`
    // vector contains the model-state _before evaluation_ of a certain nodes.
    //
    // The models stack is offset by `first_unexecuted`, so that `models[0]`
    // always corrresponds to model state before execution of the
    // `first_unexecuted` node. When not in use, the models stack is cleared.
    //
    // When an error is detected in the plan, it is assumed to be at the node
    // with index `first_unexecuted + models.len()`.
    //
    nodes: Vec<Node<D, C::Resolver>>,
    // The index of the first node in the plan that has not been sucesfully executed.
    first_unexecuted: usize,
    // The index of the primitive action node which we are waiting to execute.
    current_action: Option<usize>,
    // True when the plan has been completed.
    complete: bool,
    // True when the plan is not currently valid and cannot be executed.
    failed: bool,
}

/// The outcome of running the planner.
#[derive(PartialEq, Debug)]
pub enum Outcome<D> {
    /// This action should be executed. Call `run` again when it is finished to
    /// get a new action.
    Action(TaskRef<D>),
    /// The plan was completed. Calling `run` again will create a fresh plan and
    /// return the first action in that plan.
    Complete,
    /// The planner was unable to generate a valid plan for the current
    /// situation.
    Fail,
    /// The planner encountered an error during planning.
    Error(PlanError),
}

#[derive(Copy, Clone, PartialEq, Debug)]
pub enum PlanError {
    TaskNotFound,
}

impl<M, D, P, C> Plan<M, D, P, C>
where
    D: Clone,
    M: Clone,
    P: PrimitiveTask<M, D>,
    C: CompoundTask<M, D>,
{
    /// Creates a new plan.
    pub fn new(tasks: Arc<EstrMap<Task<P, C>>>, root_ref: TaskRef<D>) -> Self {
        let root_node = Node {
            parent: 0,
            task_name: root_ref.name,
            task_data: root_ref.data,
            resolver: None,
            sub_tasks: vec![],
        };
        Plan {
            tasks,
            nodes: vec![root_node],
            first_unexecuted: 0,
            current_action: None,
            complete: false,
            failed: false,
        }
    }

    /// Informs the planner that the current action was executed succesfully,
    /// and requests a new action to try to perform.
    pub fn get_next(&mut self, model: M) -> Outcome<D> {
        if self.failed {
            Outcome::Fail
        } else if self.complete {
            Outcome::Complete
        } else if let Some(current_action) = self.current_action {
            self.current_action = None;
            self.first_unexecuted = current_action + 1;
            self.check_and_advance(model)
        } else {
            // If we have not yet made a plan, make one now
            let models = ModelStack::new(model);
            self.make_plan(models)
        }
    }

    /// Informs the planner that the current action could not be executed, and
    /// requests an alternative action to perform instead.
    pub fn get_alternative(&mut self, model: M) -> Outcome<D> {
        let models = ModelStack::new(model);
        if self.failed {
            Outcome::Fail
        } else if self.complete {
            Outcome::Complete
        } else if let Some(current_action) = self.current_action {
            self.current_action = None;
            self.replan_from(current_action, models)
        } else {
            self.make_plan(models)
        }
    }

    /// Resets a plan after completion or failure.
    pub fn reset(&mut self) {
        // Remove everything but the root node.
        self.nodes.truncate(1);
        // Reset the root node's state.
        self.nodes[0].resolver = None;
        self.nodes[0].sub_tasks.clear();
        // Reset the counters for executed tasks.
        self.first_unexecuted = 0;
        self.current_action = None;
        // Set completed and failed back to false.
        self.complete = false;
        self.failed = false;
    }

    /// Advances `current_action` to the next action in the plan.
    fn advance(&mut self) -> Outcome<D> {
        for i in self.first_unexecuted..self.nodes.len() {
            let node = &self.nodes[i];
            if node.resolver.is_none() {
                self.current_action = Some(i);
                return Outcome::Action(TaskRef {
                    name: node.task_name,
                    data: node.task_data.clone(),
                });
            }
        }
        // Or if there is none, return completed
        self.complete = true;
        Outcome::Complete
    }

    /// Advances to the next action, and checks that the plan is expected to succeed.
    fn check_and_advance(&mut self, model: M) -> Outcome<D> {
        let mut models = ModelStack::new(model);
        let mut next_action = None;
        // Check that the plan is valid, assuming that we approach the next
        // primitive action given the provided model.
        for i in self.first_unexecuted..self.nodes.len() {
            // Check that the node is valid
            let node = &self.nodes[i];
            if let Some(Task::Primitive(primitive)) = self.tasks.get(&node.task_name) {
                if primitive.is_valid(&node.task_data, models.read()) {
                    primitive.apply_effects(&node.task_data, models.mutate(i));

                    if next_action.is_none() {
                        next_action = Some(i);
                    }
                } else {
                    return self.replan_from(i, models);
                }
            }
        }

        // At this point, we know the plan is valid. Return the new current action.
        if let Some(next_action) = next_action {
            // Advance the index to indicate that more of the plan
            // has been executed.
            self.current_action = Some(next_action);
            let node = &self.nodes[next_action];
            return Outcome::Action(TaskRef {
                name: node.task_name,
                data: node.task_data.clone(),
            });
        } else {
            self.complete = true;
            return Outcome::Complete;
        }
    }

    /// Recreates a portion of the plan.
    ///
    /// Expects the index of a node as input. This node will be removed and an
    /// alternative plan will be generated.
    fn replan_from(&mut self, index: usize, mut models: ModelStack<M>) -> Outcome<D> {
        // The plan is a list, divided into several sections:
        // + [0, next_unexecuted) contains nodes that have been sucesfully executed
        // + [next_unexeucted, current_action] contains nodes that are being attempted currently
        // + (current_action, models.len()) contains nodes that have not yet been attempted
        //
        //
        self.unrefine_plan(index);
        self.backtrack(&mut models);
        self.make_plan(models)
    }

    /// Tries to produce a valid plan. Returns true on success and false on
    /// failure.
    fn make_plan(&mut self, mut models: ModelStack<M>) -> Outcome<D> {
        loop {
            if self.failed {
                return Outcome::Fail;
            }

            // The plan is built incrementally by checking and expanding the
            // last plan-node.
            let i = self.nodes.len() - 1;
            let node = &mut self.nodes[i];

            // Try to look up the task for the last node.
            let Some(task) = self.tasks.get(&node.task_name) else {
                return Outcome::Error(PlanError::TaskNotFound);
            };

            // Determine if the node is valid in this part of the plan.
            let valid = match task {
                // If the task is primtive, check that it is valid under the
                // current model and apply it's effects. Otherwise, backtrack.
                Task::Primitive(primitive) => {
                    if primitive.is_valid(&node.task_data, models.read()) {
                        primitive.apply_effects(&node.task_data, models.mutate(i));
                        true
                    } else {
                        false
                    }
                }
                Task::Compound(compound) => {
                    if node.resolver.is_none() {
                        let mut resolver = compound.get_resolver();
                        if let Some(mut sub_tasks) =
                            resolver.next_method(&node.task_data, &models.read())
                        {
                            sub_tasks.reverse();
                            node.resolver = Some(resolver);
                            node.sub_tasks = sub_tasks;
                            true
                        } else {
                            false
                        }
                    } else {
                        true
                    }
                }
            };

            if valid {
                // Walk back up the tree looking for a node to refine. Because the
                // plan is stored in depth-first-preorder, this is a reversed
                // depth-first-preorder search.
                let mut i = self.nodes.len() - 1;
                loop {
                    if let Some(task_ref) = self.nodes[i].sub_tasks.pop() {
                        // If we find a node that has pending tasks, we pop one of
                        // those tasks and add it at the end of the tree -- this is
                        // fundamentally what causes the tree to be stored in
                        // depth-first-preorder.
                        let node = Node {
                            parent: i,
                            task_name: task_ref.name,
                            task_data: task_ref.data,
                            resolver: None,
                            sub_tasks: vec![],
                        };
                        // Since this is added at the end of the tree, this will be
                        // the node for the next iteration.
                        self.nodes.push(node);
                        break;
                    } else {
                        // If the node doesn't have pending tasks (meaning it's an
                        // action or it's been fully refined) move to the parent. If
                        // we are at the run index (the first unexecuted task) the
                        // plan is fully refined and we can return success.
                        if i == 0 {
                            return self.advance();
                        } else {
                            i -= 1;
                        }
                    }
                }
            } else {
                // If the last node was not valid, backtrack up the tree.
                self.backtrack(&mut models);
            }
        }
    }

    /// Takes a completed plan and turns it back into a partial plan, so that
    /// planning can be resumed. All nodes after the provided index are
    /// "unrefined" and pushed back into their parent's list of subtasks.
    ///
    /// After this is called, the node at `index` will be the last node in the
    /// partial plan.
    fn unrefine_plan(&mut self, index: usize) {
        // Split off plan in half, starting with the current (failed) task.
        let invalid_nodes = self.nodes.split_off(index + 1);
        // Any nodes that are children of nodes still in the main plan must be
        // added back as unrefined sub-tasks, so that they can be used in replanning.
        for node in invalid_nodes.into_iter() {
            if let Some(parent) = self.nodes.get_mut(node.parent) {
                parent.sub_tasks.push(TaskRef {
                    name: node.task_name,
                    data: node.task_data,
                });
            }
        }
    }

    /// Removes the last node in a partial plan, and backtracks to that node's
    /// parent. This is used when the last node in a plan is found to be invalid
    /// (either because it's requirements are not met during planning, or
    /// because it cannot actually be executed in practice).
    ///
    /// If the plan has already been partially run, this may cause us to remove
    /// nodes representing primitive actions that have already been completed.
    /// For this reason, this may cause `first_unexecuted` to move backwards.
    fn backtrack(&mut self, models: &mut ModelStack<M>) {
        // We can't backtrack past the root node.
        if self.nodes.len() == 1 {
            self.failed = true;
            return;
        }
        // Grab the current node.
        let parent = self.nodes.last().expect("plan must never be empty").parent;
        // Rewind the model stack to the parent.
        models.reset_to(parent);
        // Return the first unexeucted pointer to the parent.
        self.first_unexecuted = usize::min(parent, self.first_unexecuted);
        // Truncate the plan after the parent. The plan is stored in
        // depth-first-preorder, so this has the effect of removing all the
        // children of the parent.
        self.nodes.truncate(parent + 1);
        // The parent must be a compound node, so we must generate a new method
        // to replace the old (failed) one.
        let parent_node = &mut self.nodes[parent];
        if let Some(mut sub_tasks) = parent_node
            .resolver
            .as_mut()
            .unwrap()
            .next_method(&parent_node.task_data, models.read())
        {
            sub_tasks.reverse();
            parent_node.sub_tasks = sub_tasks;
        } else {
            // If the parent has no valid methods, we must backtrack further.
            self.backtrack(models);
        }
    }
}

struct ModelStack<M: Clone> {
    stack: Vec<(M, usize)>,
}

impl<M: Clone> ModelStack<M> {
    fn new(model: M) -> Self {
        ModelStack {
            stack: vec![(model, 0)],
        }
    }

    fn read(&self) -> &M {
        &self.stack.last().unwrap().0
    }

    fn mutate(&mut self, new_index: usize) -> &mut M {
        let (last_model, last_index) = self.stack.last().unwrap().clone();
        debug_assert!(last_index < new_index);
        self.stack.push((last_model, new_index));
        &mut self.stack.last_mut().unwrap().0
    }

    fn reset_to(&mut self, index: usize) {
        for i in self.stack.len()..=0 {
            if self.stack[i].1 > index {
                self.stack.pop();
            }
        }
    }
}

#[cfg(test)]
mod test {

    #[cfg(test)]
    extern crate std;

    use alloc::vec::Vec;
    use estr::{Estr, EstrMap, EstrSet};

    use crate::{CompoundTask, MethodResolver, Outcome, Plan, PrimitiveTask, Task, TaskRef};

    // This is a simple testing model, which is just a set of keywords. Each
    // task has an optional list of methods, a list of keywords it requires,
    // and a list of keywords it adds when run.
    #[derive(Default, Clone, Debug)]
    struct Model {
        keywords: EstrSet,
    }

    impl Model {
        fn insert(&mut self, keyword: &str) {
            self.keywords.insert(keyword.into());
        }

        fn remove(&mut self, keyword: &str) {
            self.keywords.remove(&Estr::from(keyword));
        }
    }

    #[derive(Default, PartialEq, Debug)]
    struct Action {
        requires: Vec<Estr>,
        provides: Vec<Estr>,
    }

    impl PrimitiveTask<Model, ()> for Action {
        fn is_valid(&self, _data: &(), model: &Model) -> bool {
            for requirement in &self.requires {
                if !model.keywords.contains(requirement) {
                    return false;
                }
            }
            true
        }

        fn apply_effects(&self, _data: &(), model: &mut Model) {
            for keyword in &self.provides {
                model.keywords.insert(*keyword);
            }
        }
    }

    #[derive(Default, Debug, Clone)]
    struct Methods {
        methods: Vec<Vec<Estr>>,
    }

    impl CompoundTask<Model, ()> for Methods {
        type Resolver = Self;

        fn get_resolver(&self) -> Self::Resolver {
            let mut resolver = self.clone();
            // We reverse the list so that most important ones are popped first.
            resolver.methods.reverse();
            resolver
        }
    }

    impl MethodResolver<Model, ()> for Methods {
        fn next_method(&mut self, _data: &(), _model: &Model) -> Option<Vec<TaskRef<()>>> {
            let method = self
                .methods
                .pop()?
                .into_iter()
                .map(|name| TaskRef { name, data: () })
                .collect();
            Some(method)
        }
    }

    impl From<&'static str> for TaskRef<()> {
        fn from(value: &'static str) -> Self {
            TaskRef {
                name: value.into(),
                data: (),
            }
        }
    }

    struct Tasks {
        tasks: EstrMap<Task<Action, Methods>>,
    }

    impl Tasks {
        fn new(methods: &[&[&str]]) -> Self {
            let mut tasks = EstrMap::default();
            let methods: Vec<_> = methods
                .into_iter()
                .map(|m| m.into_iter().map(|n| Estr::from(*n)).collect())
                .collect();
            let methods = Methods { methods };
            tasks.insert(Estr::from("ROOT"), Task::Compound(methods));
            Tasks { tasks }
        }

        fn add_action(&mut self, name: &str, requires: &[&str], provides: &[&str]) {
            let action = Action {
                requires: requires.into_iter().map(|n| Estr::from(*n)).collect(),
                provides: provides.into_iter().map(|n| Estr::from(*n)).collect(),
            };
            self.tasks.insert(Estr::from(name), Task::Primitive(action));
        }

        fn add_compound(&mut self, name: &str, methods: &[&[&str]]) {
            let methods: Vec<_> = methods
                .into_iter()
                .map(|m| m.into_iter().map(|n| Estr::from(*n)).collect())
                .collect();
            let methods = Methods { methods };
            self.tasks.insert(Estr::from(name), Task::Compound(methods));
        }

        fn into_plan(self) -> Plan<Model, (), Action, Methods> {
            let root_ref = TaskRef {
                name: Estr::from("ROOT"),
                data: (),
            };
            Plan::new(self.tasks, root_ref)
        }
    }

    /// Tests a simple plan, with a single action and no requirements.
    #[test]
    fn test_basic() {
        // The root has a single method with a single task, "Task-1"
        let mut tasks = Tasks::new(&[&["Action-1"]]);

        // Add "Task-1" as a primitive action with no requirements
        tasks.add_action("Action-1", &[], &[]);

        let mut plan = tasks.into_plan();
        let model = Model::default();

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Action("Action-1".into()));

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Complete);
    }

    /// Tests a simple plan with three actons
    #[test]
    fn test_123() {
        // The root has a single method with a single task, "Task-1"
        let mut tasks = Tasks::new(&[&["Action-1", "Action-2", "Action-3"]]);

        // Primitive actions
        tasks.add_action("Action-1", &[], &[]);
        tasks.add_action("Action-2", &[], &[]);
        tasks.add_action("Action-3", &[], &[]);

        let mut plan = tasks.into_plan();
        let model = Model::default();

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Action("Action-1".into()));

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Action("Action-2".into()));

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Action("Action-3".into()));

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Complete);

        // Once the plan is complete, we can't run it again until we reset it.

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Complete);

        plan.reset();

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Action("Action-1".into()));

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Action("Action-2".into()));

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Action("Action-3".into()));

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Complete);
    }

    /// Tests a simple plan which has a condition on the final action that
    /// should cause failure if not met.
    #[test]
    fn test_123_requirements() {
        // The root has a single method with a single task, "Task-1"
        let mut tasks = Tasks::new(&[&["Action-1", "Action-2", "Action-3"]]);

        // Primitive actions
        tasks.add_action("Action-1", &[], &[]);
        tasks.add_action("Action-2", &[], &[]);
        tasks.add_action("Action-3", &["A"], &[]);

        let mut plan = tasks.into_plan();
        let mut model = Model::default();

        // At first, this should fail, because the third task has an unmet
        // condition "A".
        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Fail);

        // If we insert "A" into the model, it will stay failed.
        model.insert("A");

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Fail);

        // We can reset the plan to start over from the top. This will make
        // thing succeed again.
        plan.reset();

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Action("Action-1".into()));

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Action("Action-2".into()));

        // And iff we remove it again, it should stop working.
        model.remove("A");

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Fail);

        // Adding it back and resetting lets us run the plain again.
        model.insert("A");
        plan.reset();

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Action("Action-1".into()));

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Action("Action-2".into()));

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Action("Action-3".into()));

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Complete);
    }

    /// Tests the ability to re-plan in place.
    #[test]
    fn test_basic_reentance() {
        let mut tasks = Tasks::new(&[&["Branch"]]);

        tasks.add_compound(
            "Branch",
            &[
                &["Primary"],  // At first, this will plan the "Primary" action
                &["Fallback"], // If "Primary" fails, it will plan the "Fallback" action
            ],
        );

        tasks.add_action("Primary", &[], &[]);
        tasks.add_action("Fallback", &[], &[]);

        let mut plan = tasks.into_plan();
        let model = Model::default();

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Action("Primary".into()));

        // We call `get_alternative` here to indicate that "Primary" failed.
        let outcome = plan.get_alternative(model.clone());
        assert_eq!(outcome, Outcome::Action("Fallback".into()));

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Complete);
    }

    /// Tests a basic x-shaped graph for agreement.
    #[test]
    fn test_x_aggreement() {
        let mut tasks = Tasks::new(&[&["Set A", "Branch"], &["Set B", "Branch"]]);

        tasks.add_compound("Branch", &[&["Require A"], &["Require B"]]);

        tasks.add_action("Set A", &[], &["A"]);
        tasks.add_action("Set B", &[], &["B"]);

        tasks.add_action("Require A", &["A"], &[]);
        tasks.add_action("Require B", &["B"], &[]);

        let mut plan = tasks.into_plan();
        let mut model = Model::default();

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Action("Set A".into()));

        model.insert("A");

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Action("Require A".into()));

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Complete);
    }

    // Tests back-tracking across an x-shapred graph.
    #[test]
    fn test_x_misprediction() {
        let mut tasks = Tasks::new(&[&["Set A", "Branch"], &["Set B", "Branch"]]);

        tasks.add_compound("Branch", &[&["Require A"], &["Require B"]]);

        tasks.add_action("Set A", &[], &["A"]);
        tasks.add_action("Set B", &[], &["B"]);

        tasks.add_action("Require A", &["A"], &[]);
        tasks.add_action("Require B", &["B"], &[]);

        let mut plan = tasks.into_plan();
        let mut model = Model::default();

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Action("Set A".into()));

        // If we don't insert "A" here, we should backtrack to "Set B"

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Action("Set B".into()));

        // If we don't insert "B" here, we should fail

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Fail);

        // If we actually get "B" when we were expecint "A", we can continue
        // gracefully

        plan.reset();

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Action("Set A".into()));

        model.insert("B");

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Action("Require B".into()));

        let outcome = plan.get_next(model.clone());
        assert_eq!(outcome, Outcome::Complete);
    }
}
