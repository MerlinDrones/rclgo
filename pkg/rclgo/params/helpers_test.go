package params

import "github.com/merlindrones/rclgo/pkg/rclgo"

// newBareManager returns a manager without a node, publisher or services, with
// the store's change notifications wired as NewManager wires them.
func newBareManager() *Manager { return newBareManagerFor(nil) }

// newBareManagerFor is newBareManager for node n (used for matching override
// sections and logging).
func newBareManagerFor(n *rclgo.Node) *Manager {
	m := &Manager{st: newStore(), n: n}
	m.st.notify = m.onChange
	return m
}
