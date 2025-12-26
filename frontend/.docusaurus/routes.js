import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/ur/auth/Profile',
    component: ComponentCreator('/ur/auth/Profile', '9ab'),
    exact: true
  },
  {
    path: '/ur/auth/Questionnaire',
    component: ComponentCreator('/ur/auth/Questionnaire', 'd97'),
    exact: true
  },
  {
    path: '/ur/auth/SignIn',
    component: ComponentCreator('/ur/auth/SignIn', '221'),
    exact: true
  },
  {
    path: '/ur/auth/SignUp',
    component: ComponentCreator('/ur/auth/SignUp', '479'),
    exact: true
  },
  {
    path: '/ur/markdown-page',
    component: ComponentCreator('/ur/markdown-page', 'c0a'),
    exact: true
  },
  {
    path: '/ur/profile',
    component: ComponentCreator('/ur/profile', '794'),
    exact: true
  },
  {
    path: '/ur/signin',
    component: ComponentCreator('/ur/signin', '06f'),
    exact: true
  },
  {
    path: '/ur/signup',
    component: ComponentCreator('/ur/signup', '64a'),
    exact: true
  },
  {
    path: '/ur/docs',
    component: ComponentCreator('/ur/docs', '886'),
    routes: [
      {
        path: '/ur/docs',
        component: ComponentCreator('/ur/docs', '258'),
        routes: [
          {
            path: '/ur/docs',
            component: ComponentCreator('/ur/docs', '30e'),
            routes: [
              {
                path: '/ur/docs/hardware-requirements',
                component: ComponentCreator('/ur/docs/hardware-requirements', '3fc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/intro',
                component: ComponentCreator('/ur/docs/intro', '8a7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/learning-outcomes',
                component: ComponentCreator('/ur/docs/learning-outcomes', 'e7e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/personalization-example',
                component: ComponentCreator('/ur/docs/personalization-example', 'da9'),
                exact: true
              },
              {
                path: '/ur/docs/placeholder',
                component: ComponentCreator('/ur/docs/placeholder', '7c6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/quarter-overview',
                component: ComponentCreator('/ur/docs/quarter-overview', '4d4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/weeks-01-02/',
                component: ComponentCreator('/ur/docs/weeks-01-02/', '724'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/ur/docs/weeks-01-02/module-details',
                component: ComponentCreator('/ur/docs/weeks-01-02/module-details', '9fc'),
                exact: true
              },
              {
                path: '/ur/docs/weeks-03-04/',
                component: ComponentCreator('/ur/docs/weeks-03-04/', '1b4'),
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
    path: '/ur/',
    component: ComponentCreator('/ur/', 'f17'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
