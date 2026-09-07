"""Robot-local decentralized coordination primitives.

Modules in this package operate only on a robot's own state and its local world
model, which is populated through peer messages. Imports remain explicit to
keep domain-message dependencies acyclic.
"""