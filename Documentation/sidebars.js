/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  docsSidebar: [
    {type: 'doc', id: 'index', label: 'Home'},
    'what-is-jmpc',
    {type: 'doc', id: 'examples/index', label: 'Examples & getting started'},
    {
      type: 'category',
      label: 'Concepts',
      collapsed: false,
      items: [
        'concepts/nomenclature',
        'concepts/model-predictive-control',
        'concepts/quadratic-programming',
      ],
    },
    {
      type: 'category',
      label: 'API reference',
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'State-space models',
          link: {type: 'doc', id: 'api/state-space/index'},
          items: [
            'api/state-space/jss',
            'api/state-space/sim',
            'api/state-space/augment',
            'api/state-space/c2dd',
            'api/state-space/set-measured-dist',
            'api/state-space/set-unmeasured-out',
            'api/state-space/set-linearization',
            'api/state-space/overload',
          ],
        },
        {
          type: 'category',
          label: 'Nonlinear models',
          link: {type: 'doc', id: 'api/nonlinear/index'},
          items: [
            'api/nonlinear/jnl',
            'api/nonlinear/sim',
            'api/nonlinear/odesim',
            'api/nonlinear/linearize',
            'api/nonlinear/set-solver',
          ],
        },
        {
          type: 'category',
          label: 'MPC controllers',
          link: {type: 'doc', id: 'api/mpc/index'},
          items: [
            'api/mpc/jmpc',
            'api/mpc/jmpcset',
            'api/mpc/sim',
            'api/mpc/mpcsolve',
            'api/mpc/plot',
            'api/mpc/compare',
            'api/mpc/get-mpc-toolbox-obj',
          ],
        },
        {
          type: 'category',
          label: 'Simulation options',
          link: {type: 'doc', id: 'api/simulation-options/index'},
          items: [
            'api/simulation-options/jsim',
            'api/simulation-options/removebias',
          ],
        },
        {type: 'doc', id: 'api/solvers', label: 'QP solvers'},
        {
          type: 'category',
          label: 'Graphical interface',
          link: {type: 'doc', id: 'api/gui/index'},
          items: [
            'api/gui/overview',
            'api/gui/examples',
            'api/gui/auto-setup',
            'api/gui/lti',
            'api/gui/jnl',
            'api/gui/load-save',
            'api/gui/options',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Case studies',
      collapsed: true,
      items: [
        'case-studies/servo',
        'case-studies/quad-tank',
        'case-studies/dof-heli',
        'case-studies/cstr',
        'case-studies/distil',
      ],
    },
    {
      type: 'category',
      label: 'Simulink',
      collapsed: true,
      items: ['simulink/blockset', 'simulink/3d-animation'],
    },
    {
      type: 'category',
      label: 'Embedded MPC',
      collapsed: true,
      link: {type: 'doc', id: 'embedded/index'},
      items: [
        'embedded/code-gen',
        'embedded/mem-est',
        'embedded/code-verify',
        'embedded/pil',
        'embedded/dof-heli',
      ],
    },
    {
      type: 'category',
      label: 'Project information',
      collapsed: true,
      items: ['download', 'citation', 'faq', 'license'],
    },
  ],
};

module.exports = sidebars;
