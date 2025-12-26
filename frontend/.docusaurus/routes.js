import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/auth/Profile',
    component: ComponentCreator('/auth/Profile', '238'),
    exact: true
  },
  {
    path: '/auth/Questionnaire',
    component: ComponentCreator('/auth/Questionnaire', '3e8'),
    exact: true
  },
  {
    path: '/auth/SignIn',
    component: ComponentCreator('/auth/SignIn', 'd1b'),
    exact: true
  },
  {
    path: '/auth/SignUp',
    component: ComponentCreator('/auth/SignUp', 'ae6'),
    exact: true
  },
  {
    path: '/markdown-page',
    component: ComponentCreator('/markdown-page', '3d7'),
    exact: true
  },
  {
    path: '/profile',
    component: ComponentCreator('/profile', '90e'),
    exact: true
  },
  {
    path: '/signin',
    component: ComponentCreator('/signin', 'ba0'),
    exact: true
  },
  {
    path: '/signup',
    component: ComponentCreator('/signup', '312'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '341'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '9ff'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '8f2'),
            routes: [
              {
                path: '/docs/hardware-requirements',
                component: ComponentCreator('/docs/hardware-requirements', '663'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '61d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/learning-outcomes',
                component: ComponentCreator('/docs/learning-outcomes', '769'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/personalization-example',
                component: ComponentCreator('/docs/personalization-example', '8cf'),
                exact: true
              },
              {
                path: '/docs/placeholder',
                component: ComponentCreator('/docs/placeholder', '9f0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/quarter-overview',
                component: ComponentCreator('/docs/quarter-overview', '360'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/weeks-01-02/',
                component: ComponentCreator('/docs/weeks-01-02/', '72a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/weeks-01-02/module-details',
                component: ComponentCreator('/docs/weeks-01-02/module-details', '1f3'),
                exact: true
              },
              {
                path: '/docs/weeks-03-04/',
                component: ComponentCreator('/docs/weeks-03-04/', '0be'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', 'e5f'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
