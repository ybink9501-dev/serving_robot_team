import type { RouteObject } from 'react-router-dom'
import { AppLayout } from './components/layout/AppLayout'
import { DashboardPage } from './pages/DashboardPage'
import { MapSetupPage } from './pages/MapSetupPage'
import { NotFoundPage } from './pages/NotFoundPage'
import { RobotControlPage } from './pages/RobotControlPage'
import { PersonalControlPage } from './pages/PersonalControlPage'

export const routes: RouteObject[] = [
  {
    path: '/',
    element: <AppLayout />,
    children: [
      { index: true, element: <DashboardPage /> },
      { path: 'map', element: <MapSetupPage /> },
      { path: 'robots', element: <RobotControlPage /> },
      { path: 'personal', element: <PersonalControlPage /> },
      { path: '*', element: <NotFoundPage /> },
    ],
  },
]
